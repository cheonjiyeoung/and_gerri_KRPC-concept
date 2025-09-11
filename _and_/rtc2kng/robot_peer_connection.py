import asyncio
import json
import traceback
import contextlib
import re
from typing import Any, Dict, Optional

import aiortc
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCIceCandidate,
    RTCRtpSender,
)
from aiortc.rtcrtpparameters import RTCRtpEncodingParameters
from aiortc.sdp import candidate_from_sdp
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), '../..')))

def timestamp() -> str:
    """Return an ISO-like timestamp (delegates to the shared *time_sync* util)."""

    from utils.time_sync_manager import time_sync  # local import to avoid cycles
    return time_sync.timestamp()


class RobotPeerConnectionHandler:
    """Wraps aiortc signalling, media tracks, and data channels for a *robot ⇄ operator* pair."""

    # Default bitrate (bps) applied when creating transceiver send_encodings
    DEFAULT_TARGET_BITRATE = 4_000_000  # 4 Mbps

    def __init__(
            self,
            *,
            robot_id: str,
            operator_id: str,
            ws,
            video_track: Dict[str, Any],
            audio_track: Optional[Any],
            audio_player: Optional[Any],
            loop: Optional[asyncio.AbstractEventLoop],
            on_disconnect: Optional[Any] = None,
            stats_log_interval: int = 5,
    ) -> None:
        # ─ IDs
        self.robot_id = robot_id
        self.operator_id = operator_id

        # ─ I/O
        self.ws = ws
        self.video_track = video_track  # dict: { label: MediaStreamTrack }
        self.audio_track = audio_track
        self.audio_player = audio_player
        self.on_disconnect = on_disconnect

        # ─ Async infra
        self.loop = loop or asyncio.get_event_loop()
        self.stats_log_interval = stats_log_interval
        self.stats_logging_task: Optional[asyncio.Task] = None

        # ─ Print aiortc version for troubleshooting
        print(f"[{timestamp()}] 🧐 aiortc version = {aiortc.__version__}")

        # ─ WebRTC objects
        self.pc = RTCPeerConnection()
        self.data_channel = self.pc.createDataChannel("chat")

        # ─ Add video/audio transceivers with explicit bitrate settings (multi-video)
        self._add_transceivers(self.video_track, self.audio_track)

        # ─ Register event handlers and start periodic stats logging
        self._register_events()
        self._start_periodic_stats_logging()

        # ─ Expose send-helper via pub-sub
        pub.subscribe(self.send_message, "send_message")

    # ──────────────────────────────────────────────────────────────────────────────
    # Track & codec management via transceivers (multi-video support)
    # ──────────────────────────────────────────────────────────────────────────────

    def _add_transceivers(
            self,
            video_track: Dict[str, Any],  # video_track은 {레이블: 트랙객체} 형태의 딕셔너리
            audio_track: Optional[Any],
    ) -> None:

        for label, track_object in video_track.items():
            print(f"[{timestamp()}] 🎥 Adding video transceiver for track '{label}'")
            try:
                self.pc.addTransceiver(
                    track_object,
                    direction="sendrecv"
                )
            except Exception as e:
                print(f"[{timestamp()}] ❌ Error adding transceiver for video track '{label}': {e}")
                traceback.print_exc()

        if audio_track is not None:
            print(f"[{timestamp()}] 🔊 Adding audio transceiver")
            try:
                self.pc.addTransceiver(
                    audio_track,
                    direction="sendrecv",
                )
            except Exception as e:
                print(f"[{timestamp()}] ❌ Error adding transceiver for audio track: {e}")
                traceback.print_exc()

    @staticmethod
    def _prefer_codecs(transceiver) -> None:
        """
        Set AV1→VP8→H.264 preference if available in local capabilities.
        Called before creating offer so that SDP encodes preferred order.
        """
        if transceiver.kind != "video":
            return

        try:
            caps = RTCRtpSender.getCapabilities("video")
            if not caps or not caps.codecs:
                print(f"[{timestamp()}] ⚠️ Could not get video codec capabilities.")
                return
        except Exception as e:
            print(f"[{timestamp()}] ⚠️ Error getting video codec capabilities: {e}")
            return

        all_codecs = caps.codecs

        # 선호 코덱 순서 정의 (MIME 타입 소문자로 비교)
        # preferred_mime_types = ["video/av1", "video/h264", "video/vp8"]
        # preferred_mime_types = ["video/av1", "video/h264"]
        preferred_mime_types = ["video/av1", "video/vp8", "video/h264"]

        ordered_codecs = []
        # 사용 가능한 코덱 중에서 선호 코덱을 찾아 순서대로 추가
        for mime_type in preferred_mime_types:
            found_codec = next((c for c in all_codecs if c.mimeType.lower() == mime_type), None)
            if found_codec:
                ordered_codecs.append(found_codec)

        # 선호 코덱 목록에 없는 나머지 코덱들도 추가 (선택 사항, 순서 유지를 위해)
        # for c in all_codecs:
        #     if c not in ordered_codecs:
        #         ordered_codecs.append(c)

        if ordered_codecs:
            try:
                transceiver.setCodecPreferences(ordered_codecs)
                print(f"[{timestamp()}] ✅ Codec preference applied: {[c.mimeType for c in ordered_codecs]}")
            except Exception as e:
                print(f"[{timestamp()}] ❌ Error setting codec preferences: {e}")
        else:
            print(f"[{timestamp()}] ⚠️ No AV1/VP8/H.264 in capabilities or no codecs found – leaving default order")

    def _apply_bitrate_to_sdp(self, sdp: str, target_bitrate: int) -> str:
        """
        Insert bitrate constraints and Google-specific fmtp parameters into each m=video block of the SDP,
        in a codec-agnostic but aiortc-compatible way (supports H264, VP8, VP9).

        - b=TIAS:<target_bitrate> (bits per second)
        - a=fmtp:<pt> x-google-min-bitrate=100000; x-google-max-bitrate=<target_bitrate>; x-google-start-bitrate=2000000;

        If no supported codec (H264/VP8/VP9) is found in a block, only b=TIAS is inserted.
        """
        lines = sdp.splitlines(keepends=True)
        result = []
        i = 0
        while i < len(lines):
            line = lines[i]
            if line.startswith("m=video"):
                # 1) Copy the "m=video" line
                result.append(line)

                # 2) Extract payload types from the m=video header
                parts = line.split()
                # m=video <port> <proto> <fmt> ...
                payload_types = parts[3:]

                # 3) Scan subsequent lines in this media block to find a supported codec’s pt
                pt_to_use = None
                j = i + 1
                while j < len(lines) and not lines[j].startswith("m="):
                    blk_line = lines[j]
                    result.append(blk_line)

                    # Match lines like: a=rtpmap:<pt> <codec_name>/...
                    m = re.match(r"a=rtpmap:(\d+)\s+([^\s/]+)/", blk_line, flags=re.IGNORECASE)
                    if m and pt_to_use is None:
                        pt = m.group(1)
                        codec = m.group(2).upper()
                        # Only use fmtp if codec is one of H264, VP8, VP9 and pt is in payload_types
                        if codec in ("H264", "VP8", "VP9") and pt in payload_types:
                            pt_to_use = pt
                    j += 1

                # 4) After copying existing block lines, insert fmtp if a supported pt was found
                if pt_to_use:
                    fmtp_line = (
                        f"a=fmtp:{pt_to_use} "
                        f"x-google-min-bitrate=100000;"
                        f"x-google-max-bitrate={target_bitrate};"
                        f"x-google-start-bitrate=2000000;\r\n"
                    )
                    result.append(fmtp_line)

                # 5) Insert the TIAS line for bitrate regardless
                result.append(f"b=TIAS:{target_bitrate}\r\n")

                # 6) Continue from the next media section or end
                i = j
            else:
                # Not an m=video line – copy as-is
                result.append(line)
                i += 1

        return "".join(result)

    # ──────────────────────────────────────────────────────────────────────────────
    # Offer / Answer flow (ensuring codec prefs for all video transceivers)
    # ──────────────────────────────────────────────────────────────────────────────

    async def create_and_send_offer(self) -> None:
        """Generate local offer, apply tweaks, then send via signaling WS."""
        # 1) Apply codec preference (AV1 → VP8) for every video transceiver
        for transceiver in self.pc.getTransceivers():
            if transceiver.kind == "video":
                self._prefer_codecs(transceiver)

        # 2) Create SDP offer (do not setLocalDescription yet)
        offer = await self.pc.createOffer()
        original_sdp = offer.sdp

        # 3) Insert b=TIAS lines into every m=video section
        modified_sdp = self._apply_bitrate_to_sdp(original_sdp, self.DEFAULT_TARGET_BITRATE)

        # 4) Set modified SDP as local description
        modified_offer = RTCSessionDescription(sdp=modified_sdp, type=offer.type)
        await self.pc.setLocalDescription(modified_offer)

        await self.ws.send(
            json.dumps(
                {
                    "type": "offer",
                    "sdp": self.pc.localDescription.sdp,
                    "sender": self.robot_id,
                    "receiver": self.operator_id,
                }
            )
        )
        print(f"[{timestamp()}] 📨 Offer sent {self.robot_id} → {self.operator_id}")

    async def create_answer(self) -> Optional[RTCSessionDescription]:
        """Create a local answer once remote offer is set."""
        try:
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)
            print(f"[{timestamp()}] 📝 Answer generated")
            return answer
        except Exception as exc:
            print(f"[{timestamp()}] ❌ Answer generation failed: {exc}")
            return None

    async def set_remote_description(self, sdp: str, type_: str) -> None:
        await self.pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=type_))
        print(f"[{timestamp()}] 📩 Remote description set ({type_}) ← {self.operator_id}")

    # ──────────────────────────────────────────────────────────────────────────────
    # ICE Candidate handling
    # ──────────────────────────────────────────────────────────────────────────────

    async def add_ice_candidate(self, payload: Dict[str, Any]) -> None:
        """
        Accepts both nested {candidate:{…}} and flat payloads.
        Parses `candidate_from_sdp`, builds RTCIceCandidate, and adds to PC.
        """
        # Flatten if necessary
        cand_dict = payload.get("candidate") if isinstance(payload.get("candidate"), dict) else payload
        if not isinstance(cand_dict, dict):
            print(f"[{timestamp()}] ⚠️ Invalid candidate payload: {payload}")
            return

        c_text = cand_dict.get("candidate")
        if not isinstance(c_text, str):
            print(f"[{timestamp()}] ⚠️ Candidate text missing in payload: {cand_dict}")
            return

        parsed = candidate_from_sdp(c_text)
        candidate = RTCIceCandidate(
            foundation=parsed.foundation,
            component=parsed.component,
            priority=parsed.priority,
            protocol=parsed.protocol.lower(),
            ip=parsed.ip,
            port=parsed.port,
            type=parsed.type,
            sdpMid=str(cand_dict.get("sdpMid")),
            sdpMLineIndex=int(cand_dict.get("sdpMLineIndex", 0)),
        )
        try:
            await self.pc.addIceCandidate(candidate)
            print(f"[{timestamp()}] ✅ ICE candidate added ← {self.operator_id}")
        except Exception as exc:
            print(f"[{timestamp()}] ❌ ICE add failed: {exc}")
            traceback.print_exc()

    # ──────────────────────────────────────────────────────────────────────────────
    # Event registration & stats
    # ──────────────────────────────────────────────────────────────────────────────

    def _register_events(self) -> None:
        """Attach aiortc event listeners for track, ICE, connection state, data channel."""

        @self.pc.on("track")
        async def _on_track(track):
            print(f"[{timestamp()}] 📡 Incoming {track.kind} track from {self.operator_id}")
            if track.kind == "audio" and self.audio_player:
                asyncio.ensure_future(self.audio_player(track))

        @self.pc.on("icecandidate")
        async def _on_icecandidate(candidate):
            if candidate is None:
                return
            payload = {
                "type": "candidate",
                "candidate": {
                    "candidate": candidate.candidate,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                    "usernameFragment": candidate.usernameFragment,
                },
                "sender": self.robot_id,
                "receiver": self.operator_id,
            }
            await self.ws.send(json.dumps(payload))
            print(f"[{timestamp()}] 📤 ICE candidate sent → {self.operator_id}")

        @self.pc.on("connectionstatechange")
        async def _on_state_change():
            state = self.pc.connectionState
            print(f"[{timestamp()}] 📶 Connection state = {state} ({self.operator_id})")
            if state in ("failed", "disconnected", "closed"):
                await self._cleanup()

        # Data-channel echo handlers
        @self.data_channel.on("open")
        def _on_data_channel_open():
            print(f"[{timestamp()}] ✅ Data channel open → {self.operator_id}")
            self.data_channel.send("[system] connected to robot")

        @self.data_channel.on("message")
        def _on_data_channel_message(msg):
            # print(f"[{timestamp()}] 📥 {self.operator_id}: {msg}")
            # self.data_channel.send(f"echo: {msg}")
            try:
                msg_data = json.loads(msg)
                pub.sendMessage('receive_message', message=msg_data)
            except Exception as e:
                print("ERROR :", e)



    async def _periodic_stats_logger_task(self):
        """Periodically log remote-inbound RTP stats (RTT, jitter)."""
        print(f"[{timestamp()}] 📊 Start stats logger ({self.stats_log_interval}s) for {self.operator_id}")
        try:
            while True:
                if self.pc.signalingState == "closed":
                    break
                await self._log_webrtc_stats()
                await asyncio.sleep(self.stats_log_interval)
        except asyncio.CancelledError:
            pass
        finally:
            print(f"[{timestamp()}] 📊 Stats logger stopped for {self.operator_id}")

    def _start_periodic_stats_logging(self) -> None:
        if self.stats_logging_task and not self.stats_logging_task.done():
            return
        if self.loop.is_running():
            self.stats_logging_task = self.loop.create_task(
                self._periodic_stats_logger_task()
            )

    async def _log_webrtc_stats(self) -> None:
        """Collect and print minimal remote-inbound RTP stats (video/audio jitter, RTT)."""
        try:
            stats = await self.pc.getStats()
        except Exception:
            return

        video_jitter = audio_jitter = video_rtt = audio_rtt = "N/A"
        for _id, s in stats.items():
            if s.type == "remote-inbound-rtp":
                if getattr(s, "kind", None) == "video":
                    video_jitter = getattr(s, "jitter", "N/A")
                    rt = getattr(s, "roundTripTime", None)
                    video_rtt = f"{rt * 1e3:.1f} ms" if rt else "N/A"
                elif getattr(s, "kind", None) == "audio":
                    audio_jitter = getattr(s, "jitter", "N/A")
                    rt = getattr(s, "roundTripTime", None)
                    audio_rtt = f"{rt * 1e3:.1f} ms" if rt else "N/A"

        print(
            f"[{timestamp()}] 📊 Stats | video jitter={video_jitter}  video rtt={video_rtt}  "
            f"audio jitter={audio_jitter}  audio rtt={audio_rtt}"
        )

    # ──────────────────────────────────────────────────────────────────────────────
    # Messaging helpers
    # ──────────────────────────────────────────────────────────────────────────────

    def send_message(self, message: Any) -> None:
        """Send a message over the data channel, if it is open."""
        if self.data_channel.readyState != "open":
            print(f"[{timestamp()}] ⚠️ Data channel not open")
            return
        payload = json.dumps(message) if isinstance(message, dict) else str(message)
        self.loop.call_soon_threadsafe(self._safe_send, payload)

    def _safe_send(self, payload: str) -> None:
        self.data_channel.send(payload)
        print(f"[{timestamp()}] 📤 Sent: {payload}")

    # ──────────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────────────────────────────────────

    async def _cleanup(self):
        # 1) Cancel stats-logging task (if running)
        if self.stats_logging_task and not self.stats_logging_task.done():
            self.stats_logging_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.stats_logging_task

        # 2) Close the PeerConnection if not already closed
        if self.pc.signalingState != "closed":
            await self.pc.close()

        # 3) Invoke on_disconnect callback (if provided)
        if callable(self.on_disconnect):
            self.on_disconnect(self.operator_id)

        print(f"[{timestamp()}] 🛑 Cleaned up resources for {self.operator_id}")

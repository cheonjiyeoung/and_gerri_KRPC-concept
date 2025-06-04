import asyncio
import json
import logging
import threading
import time
from asyncio import AbstractEventLoop, Task
from typing import Optional, Callable, List, Any, Set, TypeAlias

from aiortc import RTCSessionDescription, RTCPeerConnection, RTCDataChannel
from aiortc.contrib.media import MediaRecorder, MediaBlackhole

from ketirtc_operator.signaling.ws_client import WebSocketClientSignaling

from _and_.keti_rtc.webrtc_operator_video_track import VideoReceiverTrack

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

logging.getLogger("aiortc").setLevel(logging.INFO)
logging.getLogger("aioice").setLevel(logging.INFO)

TopicSubscriber: TypeAlias = Callable[[str, Any], None]

MAX_QUEUE_SIZE = 100

class RemoteRobotController:
    def __init__(self, target_id, server_host, server_port, user_id, password=None,
                 recorder=None, player=None, robot_message_handler=None):
        self._event_loop: Optional[AbstractEventLoop] = None
        self._thread = None
        self._target_robot_id = target_id
        self.user_id = user_id

        self.signaling = WebSocketClientSignaling(server_host=server_host,
                                                  server_port=server_port,
                                                  username=user_id,
                                                  password=password)
        if recorder:
            self.recorder = recorder
        else:
            self.recorder = MediaBlackhole()

        if player:
            self.player = player
        else:
            self.player = None

        self.pc = None
        self.data_channel: Optional[RTCDataChannel] = None
        self.robot_message_handler = robot_message_handler
        self.connection_state_listeners: Set[Callable[[str], Any]] = set()
        self._outbound_message_queue = asyncio.Queue()
        self._sending_task: Optional[Task] = None
        self._inbound_message_queue = asyncio.Queue()
        self._inbound_message_processing_task: Optional[Task] = None

    async def _process_outbound_queue(self):
        while True:
            message = await self._outbound_message_queue.get()
            if message is None:
                break
            await self._send_json(message)

    async def _process_inbound_queue(self):
        while True:
            item = await self._inbound_message_queue.get()
            if item is None:
                break
            message, channel_name = item
            logger.info(f"processing message from {channel_name}: {message}")
            if channel_name == "websocket":
                if message["type"] == "topic":
                    msg_body = message["msg_body"]
                    topic = msg_body["topic"]
                    value = msg_body["value"]
                    self._emit_topic_received(topic=topic, value=value)
                elif message["type"] == "request":
                    pass
            elif channel_name == "webrtc":
                # webrtc channel
                pass

    def _emit_topic_received(self, topic, value, sender_id):
        pass

    def _emit_message_received(self, message, sender_id):
        pass

    def add_connection_state_listener(self, listener: Callable[[str], Any]):
        self.connection_state_listeners.add(listener)

    def remove_connection_state_listener(self, listener: Callable[[str], Any]):
        self.connection_state_listeners.discard(listener)

    @property
    def connection_state(self):
        if self.pc is None:
            return "init"
        return self.pc.connectionState

    def set_message_received_handler(self, robot_message_handler: Callable):
        self.robot_message_handler = robot_message_handler

    def publish_topic(self, topic, value):
        cor = self.signaling.publish(topic, value)
        asyncio.run_coroutine_threadsafe(cor, loop=self._event_loop)

    def send_json(self, json_obj, flush=False):
        if self.data_channel is None:
            logger.warning(f"Data channel is none")
            return
        if self.data_channel.readyState != 'open':
            logger.warning(f"Data channel is not ready to send: state = {self.data_channel.readyState}")
            return
        if self._outbound_message_queue.qsize() > MAX_QUEUE_SIZE:
            logger.warning(f'Outbound buffer overflowed. qsize={self._outbound_message_queue.qsize()}')
        self._outbound_message_queue.put_nowait(json_obj)

    async def _send_json(self, json_obj, flush=True):
        if self.data_channel is None:
            logger.warning(f"Data channel is none")
            return
        if self.data_channel.readyState != 'open':
            logger.warning(f"Data channel is not ready to send: state = {self.data_channel.readyState}")
            return
        if self.data_channel.bufferedAmount > self.data_channel.bufferedAmountLowThreshold:
            # waiting for the lower layer to flush data
            buffer_low_event = asyncio.Event()

            def on_buffered_amount_low():
                buffer_low_event.set()

            self.data_channel.on("bufferedamountlow", on_buffered_amount_low)
            await buffer_low_event.wait()

        self.data_channel.send(json.dumps(json_obj))
        if flush:
            await self._data_channel_flush()

    def is_buffered_amount_low(self):
        return self.data_channel.bufferedAmount <= self.data_channel.bufferedAmountLowThreshold

    async def _data_channel_flush(self):
        # TODO: fix me, it's not good to call a protected member _data_channel_flush of transport
        await self.data_channel.transport._data_channel_flush()

    def data_channel_flush(self):
        if self.data_channel.readyState != 'open':
            return
        asyncio.run_coroutine_threadsafe(self._data_channel_flush(), self._event_loop)

    def handle_channel(self, channel):
        logger.info(f"data channel created: {channel.label}")

        @channel.on("open")
        def on_open():
            logger.info(f"Data channel is open: {channel.label}")
            if self._sending_task is None or self._sending_task.done():
                self._sending_task = asyncio.create_task(self._process_outbound_queue())

        @channel.on("close")
        def on_close():
            logger.info(f"Data channel has been closed: {channel.label}")
            if self._sending_task:
                self._outbound_message_queue.put_nowait(None)
                self._sending_task.cancel()
                self._sending_task = None

        @channel.on("message")
        def on_message(message):
            logger.info(f"data channel message: {message}")
            if self.robot_message_handler:
                asyncio.ensure_future(self.robot_message_handler(message, channel))
            else:
                logger.info("No handler! message received:", message)

    async def run(self):
        if self._event_loop is None:
            self._event_loop = asyncio.get_running_loop()
        elif self._event_loop != asyncio.get_running_loop():
            logger.warning(f'different event loop')

        self.signaling.on("topic", self._handle_topic_from_server)
        self.signaling.on("signal", self._handle_signal_message)
        self.signaling.on("signal", self._handle_signal_message)
        signaling_task = asyncio.create_task(self.signaling.run())
        connect_task = asyncio.create_task(self.connect())
        coros = [signaling_task, connect_task]
        await asyncio.gather(*coros)

    async def _handle_topic_from_server(self, topic_name, topic_value):
        print(f'topic from server: {topic_name}, {topic_value}')
        self._emit_topic_received(topic=topic_name, value=topic_value)

    async def _handle_signal_message(self, signal_msg, sender_id=None):
        print(f'signal message from sender={sender_id}: {signal_msg}')
        signal_type = signal_msg["type"] if "type" in signal_msg else None
        if signal_type == "answer":
            logger.info("answer received")
            data = {'sdp': signal_msg['sdp'], 'type': signal_msg['type']}
            sdc = RTCSessionDescription(**data)
            await self.pc.setRemoteDescription(sdc)
            logger.info("remote description set")
            await self.recorder.start()
        else:
            logger.warning(f'signal message unhandled')

    async def _handle_command_message(self, command, ack_callback, error_callback):
        print(f"command message: {command}")
        result = "result"
        ack_callback(result)

    async def connect(self):
        pc = RTCPeerConnection()
        self.pc = pc

        @pc.on("track")
        def on_track(track):
            logger.info("Receiving %s" % track.kind)
            self.recorder.addTrack(track)
            if track.kind == "video":
                from _and_.keti_rtc.webrtc_operator_video_track import VideoReceiverTrack, video_receiver
                receiver = VideoReceiverTrack(track)
                asyncio.ensure_future(video_receiver(receiver))

        @pc.on("datachannel")
        def on_datachannel(channel):
            logger.info("on datachannel")
            self.handle_channel(channel)

        self.data_channel = pc.createDataChannel("chat")

        self.handle_channel(self.data_channel)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            connection_state = self.pc.connectionState
            await self.handle_connection_state_change(connection_state)
            if pc.connectionState == "failed":
                await pc.close()

        # pc.addTransceiver('audio', direction='recvonly')      ### NEED CHANGE TO SEND AND RECEIVE
        self.pc.addTransceiver('video', direction='recvonly')

        logger.info("creating offer ...")
        await self.pc.setLocalDescription(await self.pc.createOffer())
        logger.info("local description set")

        offer = self.pc.localDescription
        offerMessage = {
            "RobotId": self._target_robot_id,
            "sdp": offer.sdp,
            "type": offer.type,
        }
        cmd_message = {
            "cmd": "connect2robot",
            "data": offerMessage,
        }

        await self.signaling.send_json(cmd_message)
        logger.info('offer sent')

    # async def consume_signaling(self):
    #     while True:
    #         message = await self.signaling.receive()
    #         logger.info(f"signal received: {message}")
    #         if message is None:
    #             logger.info("Exiting")
    #             break
    #         if message["type"] == "answer":
    #             logger.info("answer received")
    #
    #             sdc = RTCSessionDescription(**message)
    #             await self.pc.setRemoteDescription(sdc)
    #             logger.info("remote description set")
    #             await self.recorder.start()
    #         else:
    #             self._inbound_message_queue.put_nowait((message, "websocket"))

    async def _close(self):
        await self.signaling.close()
        await self.recorder.stop()
        if self.data_channel:
            self.data_channel.close()
        await self.pc.close()

    def close(self):
        asyncio.run_coroutine_threadsafe(self._close(), loop=self._event_loop)

    async def handle_connection_state_change(self, connectionState):
        coros = [listener(connectionState) for listener in self.connection_state_listeners]
        await asyncio.gather(*coros)

    def __start_worker(self, event_loop: Optional[AbstractEventLoop] = None):
        if event_loop:
            self._event_loop = event_loop
        if self._event_loop is None:
            self._event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._event_loop)
        self._event_loop.run_until_complete(self.run())
        logger.info('cleanup...')
        self._event_loop.run_until_complete(self._close())
        logger.info('finished')

    def start_thread(self, event_loop: Optional[AbstractEventLoop] = None):
        if self._thread and self._thread.is_alive():
            logger.info(f'ConnectionHandler thread already started')
            return self._thread
        self._thread = threading.Thread(name=f'thread {self.__class__.__name__}', target=self.__start_worker, args=(event_loop,))
        self._thread.start()
        logger.info(f'A new ConnectionHandler thread is started')
        return self._thread

    def subscribe(self, topic: str, handler: TopicSubscriber):
        """subscribe to a topic from the robot"""
        cor = self.signaling.subscribe(topic, handler, remote_peer_id=self._target_robot_id)
        asyncio.run_coroutine_threadsafe(cor, loop=self._event_loop)

    def unsubscribe(self, topic: str, handler: TopicSubscriber):
        """subscribe to a topic from the robot"""
        cor = self.signaling.unsubscribe(topic, handler, remote_peer_id=self._target_robot_id)
        asyncio.run_coroutine_threadsafe(cor, self._event_loop)

async def handle_robot_message(message, channel):
    print(f"robot message received: {message}")
    logger.info(f"robot message received: {message}")


def print_commands_list():
    print(f'==================')
    print(f'Valid commands: ')
    print(f'-------------')
    valid_commands = ['quit/q', 'left/s', 'right/f', 'forward/e', 'backward/d', 'stop/space',]
    for i, cm in enumerate(valid_commands):
        print(f'{i})\t{cm}')

def get_move_command(direction):
    move_cmd = {"cmd": "command",
                "data": {
                    "cmd_type": "move",
                    "params": {
                        "direction": direction,
                    }
                }
    }
    return move_cmd

def get_stop_command():
    move_cmd = {"cmd": "command",
                "data": {
                    "cmd_type": "move",
                    "params": {
                        "direction": "stop",
                    }
                }
    }
    return move_cmd


if __name__ == '__main__':
    target_robot_id = 'Spot_test_01'
    server_host = '175.126.123.199'
    # server_host = 'localhost'
    server_port = 8180


    async def handle_robot_message(message, channel):
        print(f"robot message received: {message}")
        logger.info(f"robot message received: {message}")


    def print_commands_list():
        print(f'==================')
        print(f'Valid commands: ')
        print(f'-------------')
        valid_commands = ['quit/q', 'left/s', 'right/f', 'forward/e', 'backward/d', 'stop/space', ]
        for i, cm in enumerate(valid_commands):
            print(f'{i})\t{cm}')


    def get_move_command(direction):
        move_cmd = {"cmd": "command",
                    "data": {
                        "cmd_type": "move",
                        "params": {
                            "direction": direction,
                        }
                    }
                    }
        return move_cmd


    def get_stop_command():
        move_cmd = {"cmd": "command",
                    "data": {
                        "cmd_type": "move",
                        "params": {
                            "direction": "stop",
                        }
                    }
                    }
        return move_cmd


    # camera_recorder = MediaRecorder(f"{target_robot_id}.mp4")
    camera_recorder = MediaBlackhole()
    remote_robot = RemoteRobotController(target_robot_id, server_host, server_port, user_id="python_user1",
                                         recorder=camera_recorder)

    async def handle_connection_state_change(connection_state):
        # connection_state values: [init, connecting, connected, failed, closed]
        print(f'connection state changed to: {connection_state}')

    print(f'connection state: {remote_robot.connection_state}')

    remote_robot.add_connection_state_listener(handle_connection_state_change)
    remote_robot.set_message_received_handler(handle_robot_message)

    try:

        remote_robot.start_thread()

        while True:
            print_commands_list()
            cmd = input('Enter command: ')
            if cmd in ['quit', 'q']:
                print('stopping ...')
                break
            elif cmd in ['left', 's']:
                move_command = get_move_command("left")
                remote_robot.send_json(move_command)
            elif cmd in ['right', 'f']:
                move_command = get_move_command("right")
                remote_robot.send_json(move_command)
            elif cmd in ['forward', 'e']:
                move_command = get_move_command("forward")
                remote_robot.send_json(move_command)
            elif cmd in ['backward', 'd']:
                move_command = get_move_command("backward")
                remote_robot.send_json(move_command)
            elif cmd in ['stop', ' ']:
                move_command = get_stop_command()
                remote_robot.send_json(move_command)
            elif cmd.isdigit():
                n = int(cmd)
                for i in range(n):
                    move_command = get_stop_command()
                    remote_robot.send_json(move_command)
                    time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        # asyncio.ensure_future(remote_robot.close(), loop=loop)
        remote_robot.close()

    print('exited!')

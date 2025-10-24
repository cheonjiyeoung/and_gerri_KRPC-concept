import asyncio
import json

from zimmer.driver import Zimmer


class ZimmerDaemon:
    def __init__(self, host="192.168.3.110", port=65432):
        self.host = host
        self.port = port
        self.zimmer = Zimmer()  # ✅ 단 한 번만 생성
        self.connected = False

    async def handle_client(self, reader, writer):
        try:
            data = await reader.read(1024)
            msg = json.loads(data.decode())
            cmd = msg.get("command")
            args = msg.get("args", {})

            # ✅ Zimmer 인스턴스 재사용
            if cmd == "connect":
                self.zimmer.connect()
                self.zimmer.init()  # GUI처럼 init까지 수행
                self.connected = True
                writer.write(b"[OK] Connected to gripper\n")

            elif cmd == "grip" and self.connected:
                distance = args.get("distance", 10.0)
                self.zimmer.grip(grip_distance=float(distance))
                writer.write(f"[OK] Gripped to {distance}mm\n".encode())

            elif cmd == "release" and self.connected:
                distance = args.get("distance", 30.0)
                self.zimmer.release(release_distance=float(distance))
                writer.write(f"[OK] Released to {distance}mm\n".encode())

            elif cmd == "disconnect" and self.connected:
                self.zimmer.disconnect()
                self.connected = False
                writer.write(b"[OK] Disconnected\n")

            else:
                writer.write(b"[WARN] Invalid command or not connected\n")

            await writer.drain()
        except Exception as e:
            writer.write(f"[ERROR] {str(e)}\n".encode())
        finally:
            writer.close()
            await writer.wait_closed()

    async def start(self):
        server = await asyncio.start_server(self.handle_client, self.host, self.port)
        print(f"[ZIMMERD] Daemon running on {self.host}:{self.port}")
        async with server:
            await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(ZimmerDaemon().start())
    except KeyboardInterrupt:
        print("[ZIMMERD] Server stopped manually.")

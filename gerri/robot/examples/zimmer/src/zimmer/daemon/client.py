import asyncio
import json
import os

from dotenv import load_dotenv

load_dotenv()

HOST = os.getenv("ZIMMER_DAEMON_HOST", "127.0.0.1")  # 서버 IP (현재는 동일 PC)
PORT = int(os.getenv("ZIMMER_DAEMON_PORT", "65432"))


async def send_command(command, **kwargs):
    """CLI → Daemon 통신"""
    try:
        reader, writer = await asyncio.open_connection(HOST, PORT)
        payload = json.dumps({"command": command, "args": kwargs})
        writer.write(payload.encode())
        await writer.drain()

        data = await reader.read(1024)
        print(data.decode())

        writer.close()
        await writer.wait_closed()

    except ConnectionRefusedError:
        print(
            "[ERROR] Daemon not running. Start it with `python -m zimmer.daemon.server`"
        )

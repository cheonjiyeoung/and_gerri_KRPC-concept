import click
import os
from zimmer.driver import Zimmer

@click.command("connect")
@click.option("--ip", default=lambda: os.getenv("ZIMMER_IP", "192.168.3.112"), help="Zimmer IP")
@click.option("--port", default=lambda: int(os.getenv("ZIMMER_PORT", "502")), help="Modbus TCP Port")
def connect_cmd(ip, port):
    z = Zimmer()
    z.connect(ip, port)
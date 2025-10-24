import click
from zimmer.driver import Zimmer

@click.command("disconnect")
def disconnect_cmd():
    """Disconnect from Zimmer Gripper"""
    z = Zimmer()
    z.disconnect()
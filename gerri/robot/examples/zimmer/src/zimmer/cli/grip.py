import click
import os
from zimmer.driver import Zimmer

@click.command("grip")
@click.option(
    "--distance",
    type=float,  # ← 핵심!
    default=lambda: float(os.getenv("ZIMMER_GRIP_DISTANCE", "10.0")),
    show_default=True,
    help="Grip distance (mm)"
)
def grip_cmd(distance):
    z = Zimmer()
    z.connect()
    z.grip(grip_distance=distance)
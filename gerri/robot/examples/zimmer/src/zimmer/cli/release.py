import click
import os
from dotenv import load_dotenv
from zimmer.driver import Zimmer

@click.command("release")
@click.option(
    "--distance",
    default=lambda: float(os.getenv("ZIMMER_RELEASE_DISTANCE", "30.0")),
    show_default=True,
    help="Release distance (mm)"
)
def release_cmd(distance):
    """Release gripper to specified distance"""
    z = Zimmer()
    z.connect()
    z.release(release_distance=distance)
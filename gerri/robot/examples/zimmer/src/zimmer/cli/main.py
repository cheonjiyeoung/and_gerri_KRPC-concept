import asyncio

import click
from zimmer.daemon.client import send_command


@click.group()
def cli():
    """Zimmer Gripper Command Line Interface (Async)"""
    pass


@cli.command()
def connect():
    asyncio.run(send_command("connect"))


@cli.command()
@click.option("--distance", default=10.0, help="Grip distance (mm)")
def grip(distance):
    asyncio.run(send_command("grip", distance=distance))


@cli.command()
@click.option("--distance", default=30.0, help="Release distance (mm)")
def release(distance):
    asyncio.run(send_command("release", distance=distance))


@cli.command()
def disconnect():
    asyncio.run(send_command("disconnect"))


@cli.command()
def stop():
    asyncio.run(send_command("stop"))


if __name__ == "__main__":
    cli()

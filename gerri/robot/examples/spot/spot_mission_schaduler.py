from apscheduler.schedulers.asyncio import AsyncIOScheduler
import asyncio
import datetime
from pubsub import pub
import time

class MissionSchaduler:
    def __init__(self, subcontroller,loop):
        self.subcontroller = subcontroller
        self.loop = loop

    def exec_scheduler_interval(self, mission, t_value):
        scheduler = AsyncIOScheduler(event_loop=self.loop)
        scheduler.add_job(self.run_mission, "interval", seconds=t_value, args=[mission])
        scheduler.start()

    def exec_scheduler_cron(self, mission, t_value):
        scheduler = AsyncIOScheduler(event_loop=self.loop)
        scheduler.add_job(self.subcontroller.run_mission, "cron", minute=t_value, args=[mission])
        scheduler.start()

        

class TESTROBOT:
    async def move_waypoint(self, poi):
        print(f"이동시작 ({poi})")
        await asyncio.sleep(5)
        print(f"이동완료 ({poi})")
        return True


async def main():
    robot = TESTROBOT()
    tm = MissionSchaduler(robot)
    tm.exec_scheduler_interval(mission=tm.mission_1, t_value=15)

    while True:
        print("running")
        await asyncio.sleep(5)

if __name__ == "__main__":
    asyncio.run(main())

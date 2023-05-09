#!/usr/bin/env python3
import asyncio
import sys
import argparse
from argparse import Namespace
from typing import Any, NoReturn
from mavsdk.mission import (MissionItem, MissionPlan)
from mission_helpers_module.abstract_modules.abstract_mission_helper import AbstractMissionHelper
from time import sleep
from mavsdk import System


# https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py


class Mission(AbstractMissionHelper):
    def __init__(self) -> None:
        self.home_location = None
        self.connection = "udp://:14540"
        self.mavMode = 4
        self.vehicle = None
        self.cmds = None
        self.homePosSet = False
        self.runningTask = Any
        self.ulog_filename = None
        self.mission_items = []

    def add_mission_item(self, mission_item):
        self.mission_items.append(mission_item)

    # Get arguments and connect information
    def get_arguments(self) -> Namespace:
        parser = argparse.ArgumentParser()
        parser.add_argument("-c", "--connect", help="connection string")
        args = parser.parse_args()
        if args.connect:
            self.connection = args.connect
        return args

    # Connect to the Vehicle
    async def connect(self) -> None:
        self.vehicle = System()
        await self.vehicle.connect(system_address=self.connection)
        async for state in self.vehicle.core.connection_state():
            print(state)
            if state.is_connected:
                print(f"Drone discovered")
                break

    # Meta function to reduce the number of calls needed
    async def print_connection_status(self) -> None:
        async for state in self.vehicle.core.connection_state():
            print(state)

    async def setup_vehicle(self) -> None:
        self.get_arguments()
        await self.connect()

    async def arm(self, tries=0):
        if tries >= 3:
            print("Failed to arm drone")
            sys.exit(1)

        try:
            await self.vehicle.action.arm()
        except Exception as err:
            print(f"Error: {err}")
            sleep(2)
            await self.arm(tries=tries + 1)

    async def disarm(self):
        await self.vehicle.action.disarm()
        await asyncio.sleep(5)

    async def start_mission(self):
        # for i in range(4):
        try:
            await self.vehicle.mission.start_mission()
        # return
        except Exception as err:
            print(err)
            print("Mission didn't start. ending mission")

    async def upload_mission(self, mission_plan):
        await self.vehicle.mission.upload_mission(mission_plan)
        print("Waiting for drone to have a global position estimate...")
        async for health in self.vehicle.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

    async def monitor_mission(self):
        prev_mission = -1
        async for mission_progress in self.vehicle.mission.mission_progress():

            if prev_mission == mission_progress.current:
                print("Mission Failed...")
                print("exiting")
                sys.exit(1)

            prev_mission = mission_progress.current
            print(f"Mission progress: "
                  f"{mission_progress.current}/"
                  f"{mission_progress.total}", end="\r")
            # await self.printPosition()

    async def drone_in_air(self, running_tasks):
        """ Monitors whether the drone is flying or not and
        returns after landing """

        print("Monitoring simulation started")
        was_in_air = False
        async for is_in_air in self.vehicle.telemetry.in_air():
            if is_in_air:
                was_in_air = is_in_air

            if was_in_air and not is_in_air:
                print("-- Disarming drone")
                await self.disarm()

                print("Downloading Log Entries")
                log_entries = await self.get_entries()
                for i, entry in enumerate(log_entries):
                    await self.download_log(entry=entry, suffix=str(i))
                await self.vehicle.log_files.erase_all_log_files()

                # TODO Might need to remove the following
                for task in running_tasks:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
                await asyncio.get_event_loop().shutdown_asyncgens()
                return

    async def get_home_location(self):
        home_lat, home_lon = None, None
        async for position in self.vehicle.telemetry.position():
            home_lat = position.latitude_deg
            home_lon = position.longitude_deg
            break
        return home_lat, home_lon

    async def set_home_location(self):
        async for position in self.vehicle.telemetry.position():
            self.home_location = position
            break

    async def print_status(self, verbose=False):
        # TODO: remove this
        if verbose:
            async for status_text in self.vehicle.telemetry.status_text():
                print(status_text)

    # Methods for downloading log files
    async def download_log(self, entry, suffix=""):
        if self.ulog_filename is None:
            self.ulog_filename = entry.date.replace(":", "-")
        else:
            self.ulog_filename = self.ulog_filename.rstrip(".json")

        # TODO: Allow for input of log file name

        filename = f"/root/data/sim_{suffix}_{self.ulog_filename}.ulg"
        print(f"Downloading: log {entry.id} from {entry.date} to{filename}")
        await self.vehicle.log_files.download_log_file(entry, filename)
        print("Done...")

    async def get_entries(self):
        entries = await self.vehicle.log_files.get_entries()
        for entry in entries:
            print(f"Log {entry.id} from {entry.date}")
        return entries


# https://docs.px4.io/v1.9.0/en/flight_modes/ for other types of missions
async def main() -> NoReturn:
    mission_alt = 10
    mission_spd = 10
    mission = Mission()

    print("Connecting to vehicle")
    await mission.connect()
    # termination_task = asyncio.ensure_future(mission.drone_in_air())  # keeps script running if drone in air

    home_lat, home_lon = await mission.get_home_location()
    print(f'home location\n\t>lat:{home_lat}\n\t>lon:{home_lon}')
    await mission.vehicle.mission.clear_mission()
    mission.mission_items = [MissionItem(home_lat,
                                         home_lon,
                                         mission_alt,
                                         mission_spd,
                                         is_fly_through=True,
                                         gimbal_pitch_deg=0,
                                         gimbal_yaw_deg=0,
                                         camera_action=MissionItem.CameraAction.NONE,
                                         loiter_time_s=float('nan'),
                                         camera_photo_interval_s=float('nan'),
                                         acceptance_radius_m=float('nan'),
                                         yaw_deg=float('nan'),
                                         camera_photo_distance_m=float('nan'))]
    # Takeoff
    # Setting Mission waypoints
    wp_north = mission.get_offset_location(original_location=(home_lat, home_lon), d_north=10, d_east=0)
    wp_east = mission.get_offset_location(original_location=(wp_north[0], wp_north[1]), d_north=0, d_east=10)
    wp_south = mission.get_offset_location(original_location=(wp_east[0], wp_east[1]), d_north=-10, d_east=0)

    # Setting mission waypoints
    for item in [wp_north, wp_east, wp_south]:
        new_item = MissionItem(item[0],
                               item[1],
                               mission_alt,
                               mission_spd,
                               is_fly_through=True,
                               gimbal_pitch_deg=0,
                               gimbal_yaw_deg=0,
                               camera_action=MissionItem.CameraAction.NONE,
                               loiter_time_s=float('nan'),
                               camera_photo_interval_s=float('nan'),
                               acceptance_radius_m=float('nan'),
                               yaw_deg=float('nan'),
                               camera_photo_distance_m=float('nan'))
        mission.add_mission_item(new_item)

    await mission.vehicle.mission.set_return_to_launch_after_mission(True)

    mission_plan = MissionPlan(mission.mission_items)
    print("-- Uploading mission")
    await mission.upload_mission(mission_plan)

    print("-- Arming")
    await mission.arm()

    print("-- Starting mission")
    await mission.start_mission()
    # await termination_task
    print("-- Finishing mission")


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())

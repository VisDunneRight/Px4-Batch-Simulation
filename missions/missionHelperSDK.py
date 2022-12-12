#!/usr/bin/env python3
from typing import Any, Dict, NoReturn
import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

from pymavlink import mavutil
import time
import sys
import argparse
import math

# https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py


class Mission:
    def __init__(self) -> None:
        self.connection = "udp://:14540"
        self.mavMode = 4
        self.vehicle = None
        self.cmds = None
        self.homePosSet = False
        self.runningTask = Any

    # Get arguments and connect information
    def getArguements(self) -> Dict[str, Any]:
        parser = argparse.ArgumentParser()
        parser.add_argument("-c", "--connect", help="connection string")
        args = parser.parse_args()
        if args.connect:
            self.connection = args.connect
        return args

    # Connect to the Vehicle
    async def connectVehicle(self) -> None:
        self.vehicle = System()
        await self.vehicle.connect(system_address=self.connection)
        async for state in self.vehicle.core.connection_state():
            if state.is_connected:
                print("Drone discovered!")
                break
        # printMissionProgressTask = asyncio.ensure_future(
            # self.printMissionProgress())
        # self.runningTask = [printMissionProgressTask]

    # Meta function to reduce the number of calls needed

    async def setupVehicle(self) -> None:
        self.getArguements()
        self.connectVehicle()

    async def arm(self):
        await self.vehicle.action.arm()

    async def startMission(self):
        # for i in range(4):
        # try:
        await self.vehicle.mission.start_mission()
        # return
        # except:
        # pass
        # print("Mission didn't start. ending mission")
        # await asyncio.get_event_loop().shutdown_asyncgens()
        # return

    async def uploadMission(self, missionPlan):
        await self.vehicle.mission.upload_mission(missionPlan)
        print("Waiting for drone to have a global position estimate...")
        async for health in self.vehicle.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

    def getOffsetFromLocationMeters(self, oriLat, oriLon, dNorth, dEast):
        earth_radius = 6378137.0    # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius * math.cos(math.pi * oriLat/180))

        # New position in decimal degrees
        newlat = oriLat + (dLat * 180/math.pi)
        newlon = oriLon + (dLon * 180/math.pi)
        return newlat, newlon

    async def printMissionProgress(self):
        async for mission_progress in self.vehicle.mission.mission_progress():
            print(f"Mission progress: "
                  f"{mission_progress.current}/"
                  f"{mission_progress.total}")

    async def droneInAir(self):
        """ Monitors whether the drone is flying or not and
        returns after landing """

        wasInAir = False
        async for isInAir in self.vehicle.telemetry.in_air():
            if isInAir:
                wasInAir = isInAir
            if wasInAir and not isInAir:
                # for task in self.runningTask:
                #   task.cancel()
                #   try:
                #     await task
                #   except asyncio.CancelledError:
                #     pass
                await asyncio.get_event_loop().shutdown_asyncgens()
                return

    async def getHomeLatLon(self):
        async for position in self.vehicle.telemetry.position():
            home_lat = position.latitude_deg
            home_lon = position.longitude_deg
            break
        return home_lat, home_lon


# https://docs.px4.io/v1.9.0/en/flight_modes/ for other types of missions
async def main() -> NoReturn:
    missionAlt = 10
    missionSpd = 10
    mission = Mission()

    print("Connecting to vehicle")
    await mission.connectVehicle()

    # keeps script running if drone in air
    termination_task = asyncio.ensure_future(mission.droneInAir())

    homeLat, homeLon = await mission.getHomeLatLon()
    print(f'home location\n\t>lat:{homeLat}\n\t>lon:{homeLon}')
    await mission.vehicle.mission.clear_mission()
    mission_items = []
    # Takeoff
    mission_items.append(MissionItem(homeLat,
                                     homeLon,
                                     missionAlt,
                                     missionSpd,
                                     is_fly_through=True,
                                     gimbal_pitch_deg=0,
                                     gimbal_yaw_deg=0,
                                     camera_action=MissionItem.CameraAction.NONE,
                                     loiter_time_s=float('nan'),
                                     camera_photo_interval_s=float('nan'),
                                     acceptance_radius_m=float('nan'),
                                     yaw_deg=float('nan'),
                                     camera_photo_distance_m=float('nan')))
    # Setting Mission waypoints
    wp_north = mission.getOffsetFromLocationMeters(
        homeLat, homeLon, dNorth=10, dEast=0)
    wp_east = mission.getOffsetFromLocationMeters(
        wp_north[0], wp_north[1], dNorth=0, dEast=10)
    wp_south = mission.getOffsetFromLocationMeters(
        wp_east[0], wp_east[1], dNorth=-10, dEast=0)
    # Setting mission waypoints
    mission_items.append(MissionItem(wp_north[0],
                                     wp_north[1],
                                     missionAlt,
                                     missionSpd,
                                     is_fly_through=True,
                                     gimbal_pitch_deg=0,
                                     gimbal_yaw_deg=0,
                                     camera_action=MissionItem.CameraAction.NONE,
                                     loiter_time_s=float('nan'),
                                     camera_photo_interval_s=float('nan'),
                                     acceptance_radius_m=float('nan'),
                                     yaw_deg=float('nan'),
                                     camera_photo_distance_m=float('nan')))

    mission_items.append(MissionItem(wp_east[0],
                                     wp_east[1],
                                     missionAlt,
                                     missionSpd,
                                     is_fly_through=True,
                                     gimbal_pitch_deg=0,
                                     gimbal_yaw_deg=0,
                                     camera_action=MissionItem.CameraAction.NONE,
                                     loiter_time_s=float('nan'),
                                     camera_photo_interval_s=float('nan'),
                                     acceptance_radius_m=float('nan'),
                                     yaw_deg=float('nan'),
                                     camera_photo_distance_m=float('nan')))

    mission_items.append(MissionItem(wp_south[0],
                                     wp_south[1],
                                     missionAlt,
                                     missionSpd,
                                     is_fly_through=True,
                                     gimbal_pitch_deg=0,
                                     gimbal_yaw_deg=0,
                                     camera_action=MissionItem.CameraAction.NONE,
                                     loiter_time_s=float('nan'),
                                     camera_photo_interval_s=float('nan'),
                                     acceptance_radius_m=float('nan'),
                                     yaw_deg=float('nan'),
                                     camera_photo_distance_m=float('nan')))

    await mission.vehicle.mission.set_return_to_launch_after_mission(True)

    mission_plan = MissionPlan(mission_items)
    print("-- Uploading mission")
    await mission.uploadMission(mission_plan)

    print("-- Arming")
    await mission.arm()

    print("-- Starting mission")
    await mission.startMission()

    await termination_task
    print("-- Finishing mission")


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())

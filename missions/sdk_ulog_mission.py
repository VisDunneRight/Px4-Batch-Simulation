import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from .missionHelperSDK import Mission
import json
import argparse
# from utils.ULogHelper import *


async def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("mission_path", type=str,
                        help="the path to the flight plan")

    args = parser.parse_args()

    missionAlt = 10
    missionSpd = 10
    mission = Mission()

    print("Connecting to vehicle")
    await mission.connectVehicle()

    # To show mission progress
    # t_progress = asyncio.create_task(mission.printMissionProgress())
    # print(args.mission_path)

    # keeps script running if drone in air
    termination_task = asyncio.ensure_future(mission.droneInAir())

    homeLat, homeLon = await mission.getHomeLatLon()
    print(f'home location\n\t>lat:{homeLat}\n\t>lon:{homeLon}')
    await mission.vehicle.mission.clear_mission()

    mission_items = []

    # Load x and y coordinates from ulg file.
    with open(args.mission_path, "r", encoding="utf-8") as input_data:
        uav_data = json.load(input_data)

    # for key  in coords_data:
    wp_data = uav_data["wp"]
    x_coord = wp_data["wp_x"]
    y_coord = wp_data["wp_y"]
    alt = wp_data["wp_altitude_m"]
    s = wp_data["wp_speed_ms"]
    yaw = wp_data["wp_yaw_deg"]
    i = 0
    for x, y, missionAlt, missionSpd, m_yaw in zip(x_coord, y_coord, alt, s, yaw):
        wp = mission.getOffsetFromLocationMeters(
            homeLat, homeLon, dNorth=y, dEast=x)

        if missionSpd < 0.5:
            continue

        # print(f"x:{x} | y:{y} --> speed: {missionSpd}")

        mission_items.append(MissionItem(wp[0],
                                         wp[1],
                                         missionAlt,
                                         missionSpd,
                                         is_fly_through=True,
                                         gimbal_pitch_deg=0,
                                         gimbal_yaw_deg=0,
                                         camera_action=MissionItem.CameraAction.NONE,
                                         loiter_time_s=float('nan'),
                                         camera_photo_interval_s=float('nan'),
                                         acceptance_radius_m=float('nan'),
                                         yaw_deg=m_yaw,
                                         #  yaw_deg=float('nan'),
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
    print("--Finishing mission")

if __name__ == "__main__":
    # main()
    loop = asyncio.get_event_loop()
    # loop.run_until_complete(takeoff_land(20))
    loop.run_until_complete(main())

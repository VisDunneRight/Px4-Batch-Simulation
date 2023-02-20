import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from missionHelperSDK import Mission
import json
import argparse
from time import sleep
import sys
# from utils.ULogHelper import *


async def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("mission_path", type=str,
                        help="the path to the flight plan")
    # parser.add_argument("ulog_save_path", type=str,
    #                     help="the path to save the ulog file")

    args = parser.parse_args()

    

    missionAlt = 10
    missionSpd = 10
    mission = Mission()

   
    await mission.connectVehicle()
   

    # Task to run in parallel
    print_mission_progress_task = asyncio.ensure_future(
        mission.printMissionProgress())
    print_status_task = asyncio.ensure_future(mission.printStatus())
    print_progress_task = asyncio.ensure_future(mission.printMissionProgress())

    running_tasks = [print_mission_progress_task,
                     print_status_task, print_progress_task]

    termination_task = asyncio.ensure_future(mission.droneInAir(
        running_tasks))  # keeps script running if drone in air
    
    # to save files same as downloaded ulogs from Px4 server
    mission.ulog_filename = args.mission_path.split("/")[-1]

    await mission.vehicle.mission.clear_mission()

    homeLat, homeLon = await mission.getHomeLatLon()
    print(f'home location\n\t>lat:{homeLat}\n\t>lon:{homeLon}')
    await mission.vehicle.mission.clear_mission()

    mission_items = []

    # Load x and y coordinates from ulg file.
    with open(args.mission_path, "r", encoding="utf-8") as input_data:
        uav_data = json.load(input_data)

    # for key  in coords_data:
    # wp_data = uav_data["wp"]

    x_coord = uav_data["x"]
    y_coord = uav_data["y"]
    alt = uav_data["altitude"]
    s = uav_data["velocity"]
    # yaw = uav_data["wp_yaw_deg"]

    for x, y, missionAlt, missionSpd in zip(x_coord, y_coord, alt, s):
        wp = mission.getOffsetFromLocationMeters(
            homeLat, homeLon, dNorth=y, dEast=x)

        if missionSpd < 0.5:
            continue

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
                                         #  yaw_deg=m_yaw,
                                         yaw_deg=float('nan'),
                                         camera_photo_distance_m=float('nan')))

    await mission.vehicle.mission.set_return_to_launch_after_mission(True)

    mission_plan = MissionPlan(mission_items)
    print("-- Uploading mission")
    await mission.uploadMission(mission_plan)

    print("-- Arming")
    await mission.arm()

    sleep(3)

    print("-- Starting mission")
    await mission.startMission()


    await termination_task
    print("--Finishing mission")

if __name__ == "__main__":
    # main()
    try:
        loop = asyncio.get_event_loop()
        # loop.run_until_complete(takeoff_land(20))
        loop.run_until_complete(main())
    except:
        sys.exit(1)

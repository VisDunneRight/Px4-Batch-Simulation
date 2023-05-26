import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from mission_helpers_module.px4_mission_helpers import mission_helper_sdk

try:
    from mission_helpers_module.arduino_mission_helpers import mission_helper_dronekit
except ModuleNotFoundError as error:
    print("WARN: DRONEKIT NOT INSTALLED...can't run ArduPilot without it")
import json
import argparse
from time import sleep
import sys


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mission_path", type=str,
                        help="the path to the flight plan"
                        )
    parser.add_argument("simulator", type=str,
                        help="name of the simulator being used.")
    parser.add_argument("--save-dir", type=str, help="Where to save to flight log.")
    args = parser.parse_args()
    # get the name of the mission.
    mission_name = args.mission_path.split("/")[-1].rstrip(".json")

    if args.simulator in ["JMavSim", "Gazebo"]:
        mission = mission_helper_sdk.Mission()
    elif args.simulator == "ArduPilot":
        mission = mission_helper_dronekit.Mission()
    else:
        print("Not a simulator option")
        print("CHOOSE:", ["Gazebo", "JMavSim", "ArduPilot"])
        sys.exit(1)

    mission.set_mission_name(mission_name)

    if args.save_dir:
        mission.set_log_dir(args.save_dir)
    await mission.connect()
    # to save files same as downloaded u_logs from Px4 server
    mission.ulog_filename = args.mission_path.split("/")[-1]

    home_lat, home_lon = await mission.get_home_location()
    print(f'home location\n\t>lat:{home_lat}\n\t>lon:{home_lon}')
    await mission.clear_mission()
    # # Load x and y coordinates from ulg file.
    print("Loading mission plan file...")
    with open(args.mission_path, "r", encoding="utf-8") as input_data:
        uav_data = json.load(input_data)

    x_coord = uav_data["x"]
    y_coord = uav_data["y"]
    alt = uav_data["altitude"]
    s = uav_data["velocity"]

    print("GENERATING MISSION PLAN...")
    for x, y, mission_alt, mission_spd in zip(x_coord, y_coord, alt, s):
        new_lat, new_lon = mission.get_offset_location(original_location=(home_lat, home_lon), d_north=y, d_east=x)
        if mission_spd < 0.5:
            continue
        # Have to figure out how to set speed with dronekit,
        # TODO: Tried to see how the max set velocity works
        mission_spd = max(s)
        mission.add_mission_item(latitude=new_lat, longitude=new_lon, altitude=mission_alt, speed=mission_spd)

    print("UPLOADING MISSION PLAN...")
    await mission.upload_mission()
    sleep(5)
    print("-- Arming")
    await mission.arm()
    sleep(3)

    print("-- Starting mission")
    await mission.start_mission()
    sleep(5)
    await mission.close_connection()
    print("--Finishing mission")
    await asyncio.sleep(5)
    return

if __name__ == "__main__":
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(main())
        # Get all tasks that are still running
        running_tasks = asyncio.all_tasks(loop=loop)
        # Print the running tasks
        print("mission done!")
    except Exception as err:
        print(err)
        sys.exit(1)  # Mission Fail

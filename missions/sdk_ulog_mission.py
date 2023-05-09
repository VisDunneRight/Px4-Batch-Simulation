import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from mission_helpers_module.px4_mission_helpers import mission_helper_sdk
try:
    from mission_helpers_module.arduino_mission_helpers import mission_helper_dronekit
except ModuleNotFoundError as error:
    print("DRONEKIT NOT INSTALLED...can't run ArduPilot without it")
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

    args = parser.parse_args()

    if args.simulator in ["JMavSim", "Gazebo"]:
        mission = mission_helper_sdk.Mission()
    elif args.simulator == "ArduPilot":
        mission = mission_helper_dronekit.Mission()
    else:
        print("Not a simulator option")
        print("CHOOSE:", ["Gazebo", "JMavSim", "ArduPilot"])
        sys.exit(1)

    await mission.connect()
    # Task to run in parallel
    print_mission_progress_task = asyncio.ensure_future(
        mission.monitor_mission())
    print_status_task = asyncio.ensure_future(
        mission.print_status(verbose=False))
    print_progress_task = asyncio.ensure_future(mission.monitor_mission())

    running_tasks = [print_mission_progress_task,
                     print_status_task, print_progress_task]

    termination_task = asyncio.ensure_future(mission.drone_in_air(
        running_tasks))  # keeps script running if drone in air

    # to save files same as downloaded u_logs from Px4 server
    mission.ulog_filename = args.mission_path.split("/")[-1]

    await mission.vehicle.mission.clear_mission()

    home_lat, home_lon = await mission.get_home_location()
    print(f'home location\n\t>lat:{home_lat}\n\t>lon:{home_lon}')
    await mission.vehicle.mission.clear_mission()

    # Load x and y coordinates from ulg file.
    print("Loading ulg file...")
    with open(args.mission_path, "r", encoding="utf-8") as input_data:
        print("input_data")
        uav_data = json.load(input_data)

    x_coord = uav_data["x"]
    y_coord = uav_data["y"]
    alt = uav_data["altitude"]
    s = uav_data["velocity"]
    # yaw = uav_data["wp_yaw_deg"]
    print("--CREATING MISSION PLAN")
    for x, y, mission_alt, mission_spd in zip(x_coord, y_coord, alt, s):
        print(x, y, mission_alt)

        wp = mission.get_offset_location(original_location=(home_lat, home_lon), d_north=y, d_east=x)

        if mission_spd < 0.5:
            continue
        # TODO: Remove this line later
        mission_spd = 5
        mission.add_mission_item(MissionItem(wp[0],
                                             wp[1],
                                             mission_alt,
                                             mission_spd,
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

    mission_plan = MissionPlan(mission.mission_items)
    print("-- Uploading mission")
    await mission.upload_mission(mission_plan)

    sleep(5)
    print("-- Arming")
    await mission.arm()

    sleep(5)

    print("-- Starting mission")
    await mission.start_mission()

    await termination_task
    print("--Finishing mission")


if __name__ == "__main__":
    # main()
    try:
        loop = asyncio.get_event_loop()
        # loop.run_until_complete(takeoff_land(20))
        loop.run_until_complete(main())
    except Exception as err:
        print(err)
        sys.exit(1)

import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from missionHelperSDK import Mission
import math
import random as rd

async def main():
  missionAlt = 10
  missionSpd = 10
  mission = Mission()
  
  print("Connecting to vehicle")
  await mission.connectVehicle()
  
    # Task to run in parallel
  print_mission_progress_task = asyncio.ensure_future( mission.printMissionProgress())
  print_status_task = asyncio.ensure_future(mission.printStatus())
  print_progress_task = asyncio.ensure_future(mission.printMissionProgress())
  
  # List of tasks
  running_tasks = [print_mission_progress_task, print_status_task, print_progress_task]
  
  termination_task = asyncio.ensure_future(mission.droneInAir(running_tasks)) # keeps script running if drone in air

  homeLat, homeLon = await mission.getHomeLatLon()
  print(f'home location\n\t>lat:{homeLat}\n\t>lon:{homeLon}')
  await mission.vehicle.mission.clear_mission()

  mission_items = []
  # takeoff to 10 meters  
  for i in range(4):
    wp = mission.getOffsetFromLocationMeters(homeLat, homeLon,
                                             dNorth = rd.uniform(-10, 10),
                                             dEast  = rd.uniform(-10, 10))
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
  print("--Finishing mission")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    # loop.run_until_complete(takeoff_land(20))
    loop.run_until_complete(main())

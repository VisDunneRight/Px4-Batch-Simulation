import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from missionHelperSDK import Mission
import numpy as np
import math

def calculate_figure_eight(num_points):
    # https://www.kindsonthegenius.com/plotting-tutorial-in-python-with-matplolib-pyplot-part-2/#t2
    
    # This will calculate the position for each point.
    t = np.linspace(0, 2 * np.pi, num_points)
    x = np.sin(t)
    y = (np.sin(t) * np.cos(t)) 
    
    print("DONE")
    
    return x * 10, y * 10

######################## Main function ##############################
async def main():
  missionAlt = 10
  missionSpd = 10
  mission = Mission()
  
  print("Connecting to vehicle")
  await mission.connectVehicle()
  
  termination_task = asyncio.ensure_future(mission.droneInAir()) # keeps script running if drone in air

  homeLat, homeLon = await mission.getHomeLatLon()
  print(f'home location\n\t>lat:{homeLat}\n\t>lon:{homeLon}')
  await mission.vehicle.mission.clear_mission()

  mission_items = []
  steps = 10 # number points to create the figure 8
  
  x_coords, y_coords = calculate_figure_eight(steps + 1)
  # Takeoff
 
  # takeoff to 10 meters  
  for i in range(1, steps + 1):
      
    x = x_coords[i]
    y = y_coords[i]
      
    wp = mission.getOffsetFromLocationMeters(homeLat, homeLon, dNorth=x, dEast=y)
      
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
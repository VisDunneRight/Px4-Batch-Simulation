import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from missionHelperSDK import Mission

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

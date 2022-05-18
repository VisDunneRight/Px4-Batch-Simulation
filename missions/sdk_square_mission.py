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
  # Setting Mission waypoints
  wp_north = mission.getOffsetFromLocationMeters(homeLat, homeLon, dNorth=10, dEast=0)
  wp_east = mission.getOffsetFromLocationMeters(wp_north[0], wp_north[1], dNorth=0, dEast=10)
  wp_south = mission.getOffsetFromLocationMeters(wp_east[0], wp_east[1], dNorth=-10, dEast=0)
  start = mission.getOffsetFromLocationMeters(wp_south[0], wp_south[1], dNorth=0, dEast=-10)
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

  mission_items.append(MissionItem(start[0],
                                    start[1],
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

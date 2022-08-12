import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from missionHelperSDK import Mission
import sys

async def main():
  args = sys.argv[1:]
  if(len(args) < 1):
    exit()

  mission = Mission()
  
  print("Connecting to vehicle")
  await mission.connectVehicle()
  
  termination_task = asyncio.ensure_future(mission.droneInAir()) # keeps script running if drone in air

  homeLat, homeLon = await mission.getHomeLatLon()
  print(f'home location\n\t>lat:{homeLat}\n\t>lon:{homeLon}')
  await mission.vehicle.mission.clear_mission()


  mission_import_data = await mission.vehicle.mission_raw.import_qgroundcontrol_mission(args[0])

  print(f"{len(mission_import_data.mission_items)} mission items imported")
  await mission.vehicle.mission_raw.upload_mission(mission_import_data.mission_items)
  print("Mission uploaded")

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

from typing import Any, Dict, NoReturn
import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

from pymavlink import mavutil
import time, sys, argparse, math

# https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py
class Mission:
  def __init__( self) -> None:
    self.connection = "127.0.0.1:14540"
    self.mavMode = 4
    self.vehicle = None
    self.cmds = None
    self.homePosSet = False

  #Get arguments and connect information
  def getArguements(self)-> Dict[str, Any]:
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connect", help="connection string")
    args = parser.parse_args()
    if args.connect:
      self.connection = args.connect
    return args

  #Connect to the Vehicle
  async def connectVehicle(self) -> None:
    self.vehicle = System()
    await self.vehicle.connect(system_address=self.connection)
    async for state in self.vehicle.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break


  #Meta function to reduce the number of calls needed
  async def setupVehicle(self) -> None:
    self.getArguements()
    self.connectVehicle()

  async def arm(self):
    await self.vehicle.action.arm()



  def PX4setAutoMode(self):
    self.PX4setMode(self.mavMode)

  def PX4setMode(self, mavMode):
    self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system,
                                               self.vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)


#https://docs.px4.io/v1.9.0/en/flight_modes/ for other types of missions
async def main() -> NoReturn:
  
  drone = System()
  await drone.connect(system_address="udp://:14540")

  print("Waiting for drone to connect...")
  async for state in drone.core.connection_state():
      if state.is_connected:
          print("Drone discovered!")
          break

    
  print_mission_progress_task = asyncio.ensure_future(
      print_mission_progress(drone))

  running_tasks = [print_mission_progress_task]
  termination_task = asyncio.ensure_future(
      observe_is_in_air(drone, running_tasks))


  mission_items = []
  # async for position in drone.telemetry.position():
  #       print(position)
  mission_items.append(MissionItem(47.398039859999997,
                                  8.5455725400000002,
                                  25,
                                  10,
                                  True,
                                  float('nan'),
                                  float('nan'),
                                  MissionItem.CameraAction.NONE,
                                  float('nan'),
                                  float('nan'),
                                  float('nan'),
                                  float('nan'),
                                  float('nan')))
  mission_items.append(MissionItem(47.398036222362471,
                                  8.5450146439425509,
                                  25,
                                  10,
                                  True,
                                  float('nan'),
                                  float('nan'),
                                  MissionItem.CameraAction.NONE,
                                  float('nan'),
                                  float('nan'),
                                  float('nan'),
                                  float('nan'),
                                  float('nan')))
  mission_items.append(MissionItem(47.397825620791885,
                                  8.5450092830163271,
                                  25,
                                  10,
                                  True,
                                  float('nan'),
                                  float('nan'),
                                  MissionItem.CameraAction.NONE,
                                  float('nan'),
                                  float('nan'),
                                  float('nan'),
                                  float('nan'),
                                  float('nan')))

  mission_plan = MissionPlan(mission_items)

  await drone.mission.set_return_to_launch_after_mission(True)

  print("-- Uploading mission")
  await drone.mission.upload_mission(mission_plan)

  print("-- Arming")
  await drone.action.arm()

  print("-- Starting mission")
  await drone.mission.start_mission()

  await termination_task

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return

if __name__ == '__main__':
  loop = asyncio.get_event_loop()
  loop.run_until_complete(main())
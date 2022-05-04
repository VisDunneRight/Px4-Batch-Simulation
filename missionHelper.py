# Import DroneKit-Python
from typing import Any, Dict, NoReturn
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py
class Mission:
  def __init__(self) -> None:
    self.connection = "127.0.0.1:14540"
    self.mavModeAuto = 4
    self.vehicle = None
    self.cmds = None

  #Get arguments and connect information
  def getArguements(self)-> Dict[str, Any]:
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connect", help="connection string")
    args = parser.parse_args()
    if args.connect:
      self.connection = args.connect
    return args
  
  #Connect to the Vehicle
  def connectVehicle(self) -> None:
    while 1:
      try:
          self.vehicle = connect(self.connection, wait_ready=True)
      except:
          print("Couldn't connect, trying again.")
      else:
          break
    self.cmds = self.vehicle.commands

  #Wipes commands to vechile
  def clearCommands(self) -> None:
    self.cmds.clear()

  #Gets the location of the vehicle
  def getLocation(self):
    return self.vehicle.location.global_relative_frame

  def PX4setMode(self, mavMode):
    self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system,
                                               self.vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

def main() -> NoReturn:
  print("This is a helper class for connecting to mavlink and creating missions")

if __name__ == '__main__':
    main()
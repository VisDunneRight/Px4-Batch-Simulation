# Import DroneKit-Python
from typing import Any, Dict, NoReturn
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py
class Mission:
  def __init__(self) -> None:
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
  def connectVehicle(self) -> None:
    while 1:
      try:
          self.vehicle = connect(self.connection, wait_ready=True)
      except:
          print("Couldn't connect, trying again.")
      else:
          break
    self.cmds = self.vehicle.commands
    def listener(funSelf, name, home_position):
      self.homePosSet = True
    self.vehicle.add_message_listener('HOME_POSITION', listener)
    def heartBeat(funSelf, attr_name, value):
      print(attr_name, value, end="\r")
      if value > 2:
        print("Lost connection.")
        self.reconnectVehicle()
        self.closeVehicle()
        exit(1)
    self.vehicle.add_attribute_listener('last_heartbeat', heartBeat)


  def reconnectVehicle(self):
    for i in range(5):
      try:
          self.vehicle = connect(self.connection, wait_ready=False)
      except:
          time.sleep(3)
          print("Couldn't connect, trying again.")
      else:
          break

  def waitHomePos(self):
    self.homePosSet = False
    # wait for a home position lock
    while not self.homePosSet:
        print ("Waiting for home position...")
        time.sleep(1)

  #Meta function to reduce the number of calls needed
  def setupVehicle(self) -> None:
    self.getArguements()
    self.connectVehicle()
    self.waitHomePos()
    self.status()

  def sendVhicleCommand(self, command, wp):
    return Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, command, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)

  def moveVehicleRelative(self, loc, x, y, z) -> Dict[str, Any]:
    wp = self.getOffsetFromLocationMeters(loc, x, y, z)
    cmd = self.sendVhicleCommand(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, wp)
    self.cmds.add(cmd)
    return wp
  
  def vehicleTakeOffRelative(self, loc, x, y, z) -> Dict[str, Any]:
    wp = self.getOffsetFromLocationMeters(loc, x, y, z)
    cmd = self.sendVhicleCommand(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, wp)
    self.cmds.add(cmd)
    return wp
  
  def vehicleLandRelative(self, loc, x, y, z) -> Dict[str, Any]:
    wp = self.getOffsetFromLocationMeters(loc, x, y, z)
    cmd = self.sendVhicleCommand(mavutil.mavlink.MAV_CMD_NAV_LAND, wp)
    self.cmds.add(cmd)
    return wp

  #Wipes commands to vehicle
  def clearCommands(self) -> None:
    self.cmds.clear()
  
  def uploadCommands(self) -> None:
    self.cmds.upload()
    time.sleep(2)

  #Gets the location of the vehicle
  def getLocation(self):
    return self.vehicle.location.global_relative_frame

  def arm(self):
    # self.vehicle.armed = True
    self.vehicle.arm(wait=True)

  def disarmAndWait(self):
    # self.vehicle.armed = False
    # time.sleep(1)
    self.vehicle.disarm(wait=True)

  def closeVehicle(self):
    self.vehicle.close()
    time.sleep(1)

  def status(self):
    # Display basic vehicle state
    print(" Type: %s" % self.vehicle._vehicle_type)
    print(" Armed: %s" % self.vehicle.armed)
    print(" System status: %s" % self.vehicle.system_status.state)
    print(" GPS: %s" % self.vehicle.gps_0)
    print(" Alt: %s" % self.vehicle.location.global_relative_frame.alt)

  def monitorMission(self):
    # monitor mission execution
    nextwaypoint = self.vehicle.commands.next
    while nextwaypoint < len(self.vehicle.commands):
        if self.vehicle.commands.next > nextwaypoint:
            display_seq = self.vehicle.commands.next+1
            print ("Moving to waypoint %s" % display_seq)
            nextwaypoint = self.vehicle.commands.next
        time.sleep(1)

    # wait for the vehicle to land
    while self.vehicle.commands.next > 0:
      print(self.vehicle.commands.next )
      time.sleep(1)

  def PX4setAutoMode(self):
    self.PX4setMode(self.mavMode)

  def PX4setMode(self, mavMode):
    self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system,
                                               self.vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

  def getOffsetFromLocationMeters(self, originalLocation, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `originalLocation`. The returned Location adds the entered `alt` value to the altitude of the `originalLocation`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*originalLocation.lat/180))

    #New position in decimal degrees
    newlat = originalLocation.lat + (dLat * 180/math.pi)
    newlon = originalLocation.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon, originalLocation.alt + alt)

#https://docs.px4.io/v1.9.0/en/flight_modes/ for other types of missions
def main() -> NoReturn:
  print("This is a helper class for connecting to mavlink and creating missions." 
        "A main function is provided for example purpose of simple fly in a square mission")
  
  #Example of a simple square mission
  mission = Mission()

  #Meta function to handle bookkeeping
  mission.setupVehicle()
  
  # Change to AUTO mode
  mission.PX4setAutoMode()

  mission.clearCommands()
  home = mission.getLocation()
  # takeoff to 10 meters
  wp = mission.vehicleTakeOffRelative(home, 0, 0, 10)
  # move 10 meters north
  wp = mission.moveVehicleRelative(wp, 10, 0, 0)
  # move 10 meters east
  wp = mission.moveVehicleRelative(wp, 0, 10, 0)
  # move 10 meters south
  wp = mission.moveVehicleRelative(wp, -10, 0, 0)
  # move 10 meters west
  wp = mission.moveVehicleRelative(wp, 0, -10, 0)
  #land
  wp = mission.vehicleLandRelative(home, 0, 0, 10)
  
  # Upload mission
  mission.uploadCommands()

  # Arm vehicle will excute any cmds added to the mission
  mission.arm()
  mission.monitorMission()
  mission.disarmAndWait()
  # Close vehicle object before exiting script
  mission.closeVehicle()

if __name__ == '__main__':
    main()
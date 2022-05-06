from  missionHelper import Mission

def main(): 
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
  print("upload mission")
  mission.uploadCommands()

  print("arm")
  # Arm vehicle will excute any cmds added to the mission
  mission.arm()
  print("monitor")
  mission.status()
  mission.monitorMission()
  print("disarm")
  mission.status()
  mission.disarmAndWait()
  # Close vehicle object before exiting script
  mission.closeVehicle()

if __name__ == '__main__':
    main()
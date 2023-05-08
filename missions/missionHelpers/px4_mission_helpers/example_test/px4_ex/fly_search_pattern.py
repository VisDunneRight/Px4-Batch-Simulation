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
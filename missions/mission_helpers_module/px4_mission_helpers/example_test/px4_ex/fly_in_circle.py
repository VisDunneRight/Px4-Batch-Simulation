from  missionHelper import Mission
import math

def main():
  #Example of a simple square mission
  mission = Mission()

  #Meta function to handle bookkeeping
  mission.setupVehicle()
  
  # Change to AUTO mode
  mission.PX4setAutoMode()

  mission.clearCommands()
  home = mission.getLocation()

  radius = 10 #radius of the circle in meters
  steps = 10 # number points around the circle
  # takeoff to 10 meters
  mission.vehicleTakeOffRelative(home, 0, 0, 10)
  for i in range(steps):
    angle = 2 * math.pi * float(i)/steps
    x = math.cos(angle) * radius
    y = math.sin(angle) * radius
    print(x,y)
    mission.vehicleLandRelative(home, x, y, 0)

  mission.vehicleLandRelative(home, 0, 0, 10)
  
  print("upload commands")
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
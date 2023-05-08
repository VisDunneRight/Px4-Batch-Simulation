import subprocess
import os
from time import sleep
# import psutil
# import signal
import sys

sys.path.append(os.path.dirname(os.path.dirname("./missions/ArduPilotHelper")))

COPTER_DIR = "/ardupilot/Firmware/ArduCopter"
# RUN_SIM_CMD = ["sim_vehicle.py", "-N", "-f", "gazebo-iris"]
RUN_SIM_CMD = ["sim_vehicle.py", "-N", "-v", "ArduCopter"]

def main():
    
    try:
        # Start the Ardupilt w/ Gazebo Simulator
        print("Starting Ardupilt")
        ardu_process = subprocess.Popen(RUN_SIM_CMD, stdout=subprocess.DEVNULL)
        sleep(5)
        mission_process = subprocess.run(
            ["python3", "./missions/ArduPilotHelper/missionHelperPyMav_v3.py"], check=True)
        ardu_process.terminate()
        
    except Exception as e:
        print(e)
        
   
    # try:

    #     os.chdir("/ardupilot/code")
    #     # mission_process = subprocess.run(
    #     #     ["python3", "./missions/missionHelperPyMav.py"], check=True)
    #     print("RUNNING SIMULATION...")
    #     mission_process = subprocess.run(
    #         ["python3", "./missions/ArduPilotHelper/missionHelperPyMav_v2.py", test_data], check=True)

    # except Exception as err:
    #     print("ERROR", err)

    # os.killpg(os.getpgid(0), signal.SIGTERM)


if __name__ == "__main__":
  main()
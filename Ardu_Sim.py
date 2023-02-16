import subprocess
import os
from time import sleep
import psutil
import signal
import sys

sys.path.append(os.path.dirname(os.path.dirname("./missions/ArduPilotHelper")))

COPTER_DIR = "/ardupilot/Firmware/ArduCopter"
# RUN_SIM_CMD = ["sim_vehicle.py", "-N", "-f", "gazebo-iris"]
RUN_SIM_CMD = ["sim_vehicle.py", "-N", "-v", "ArduCopter"]

RUN_GAZEBO = "gzserver"

if __name__ == "__main__":
    os.chdir(COPTER_DIR)

    print("START SERVERS...")
    gz_process = subprocess.Popen(
        [RUN_GAZEBO])
    ardu_process = subprocess.Popen(RUN_SIM_CMD, stdout=subprocess.DEVNULL)
    sleep(5)

    try:
        os.chdir("/ardupilot/code")
        # mission_process = subprocess.run(
        #     ["python3", "./missions/missionHelperPyMav.py"], check=True)

        mission_process = subprocess.run(
            ["python3", "./missions/ArduPilotHelper/missionHelperPyMav.py"], check=True)
    except Exception as err:
        print(err)

    ardu_process.kill()
    ardu_process.wait()
    processes = psutil.process_iter()

    for process in processes:
        if process.name() == "mavproxy.py" or process.name() == "arducopter":
            os.kill(process.pid, signal.SIGKILL)

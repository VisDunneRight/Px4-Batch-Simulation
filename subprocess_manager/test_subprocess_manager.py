from subprocess_manager import SubProcessManager
import shlex
import time


def test_ardupilot():
    command = shlex.split("sim_vehicle.py -v ArduCopter --console")
    print(command)
    ardu_process = SubProcessManager("ardupilot", command=command)
    ardu_process.start_process()
    print("IS RUNNING", ardu_process.is_process_running())
    while ardu_process.is_process_running():
        print(ardu_process.read_output())
        time.sleep(1)
    print("END")
    ardu_process.stop_process()
    ardu_process.wait_for_exit()
    print("IS RUNNING", ardu_process.is_process_running())


def test_gazebo():
    # Start the gazebo server.
    command = shlex.split("gzserver --verbose worlds/iris_arducopter_runway.world")
    gazebo_process = SubProcessManager("gazebo", command=command)
    gazebo_process.start_process()

    # Start Arduino Pilot.
    # command = shlex.split("sim_vehicle.py -v ArduCopter -f gazebo-iris --console")
    command = shlex.split("sim_vehicle.py -v ArduCopter --console")

    ardu_process = SubProcessManager("ardupilot", command=command)
    ardu_process.start_process()
    time.sleep(10)

    while ardu_process.is_process_running() and gazebo_process.is_process_running():
        user_input = input("Close program? enter 'C'")
        if user_input == "C":
            ardu_process.stop_process()
            gazebo_process.stop_process()
        print(ardu_process.poll_process(), end="\r")

    if ardu_process.is_process_running():
        ardu_process.stop_process()

    if gazebo_process.is_process_running():
        gazebo_process.stop_process()


if __name__ == "__main__":
    # test_ardupilot()
    test_gazebo()

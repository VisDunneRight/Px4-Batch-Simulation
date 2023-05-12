from subprocess_manager.subprocess_manager import SubProcessManager
import shlex


def build_gazebo_command(build_dir):
    pass


def build_jmavsim_command(build_dir):
    pass


def build_ardu_pilot_command():
    # Commands for Ardu and Gazebo
    gz_command = shlex.split("gzserver --verbose worlds/iris_arducopter_runway.world")
    ardu_command = shlex.split("sim_vehicle.py -v ArduCopter -f gazebo-iris --console")

    # Start gz-server
    gz_process = SubProcessManager("gz_server", command=gz_command)
    ardu_process = SubProcessManager("ardu_pilot", command=ardu_command)
    return ardu_process, gz_process


SITL_START_FUNCTIONS = {
    "Gazebo": build_gazebo_command,
    "JMavSim": build_jmavsim_command,
    "ArduPilot": build_ardu_pilot_command
}


class SimulatorManager:
    def __init__(self, simulator):
        self.simulator = simulator

        if simulator not in SITL_START_FUNCTIONS:
            raise ValueError(f"Unknown simulator {simulator}")
        self.processes = SITL_START_FUNCTIONS[simulator]()

    def start_simulator(self, build_dir=None):
        if self.simulator == "ArduPilot":
            [ardu_process, gz_process] = self.processes
            gz_process.start_process()
            ardu_process.start_process(cwd=build_dir)
        else:
            self.processes.start_process(cwd=build_dir)

    def stop_simulator(self):
        if self.simulator == "ArduPilot":
            [ardu_process, gz_process] = self.processes
            # Stop ArduPilot first...
            ardu_process.hard_stop_process()
            gz_process.hard_stop_process()
        else:
            self.processes.hard_stop_process()


from subprocess_manager.subprocess_manager import SubProcessManager
import shlex


def build_px4_command(px4_simulator):
    command = shlex.split(f"make px4_sitl {px4_simulator} HEADLESS=1")
    return SubProcessManager(px4_simulator.lower(), command=command)


def build_ardu_pilot_command():
    # Commands for Ardu and Gazebo
    gz_command = shlex.split("gzserver --verbose worlds/iris_arducopter_runway.world")
    ardu_command = shlex.split("sim_vehicle.py -v ArduCopter -f gazebo-iris --console")

    # Start gz-server
    gz_process = SubProcessManager("gz_server", command=gz_command)
    ardu_process = SubProcessManager("ardu_pilot", command=ardu_command)
    return ardu_process, gz_process


SITL_START_FUNCTIONS = {
    "Gazebo": build_px4_command("gazebo"),
    "JMavSim": build_px4_command("jmavsim"),
    "ArduPilot": build_ardu_pilot_command
}


class SimulatorManager:
    def __init__(self, simulator):
        self.simulator = simulator

        if simulator not in SITL_START_FUNCTIONS:
            raise ValueError(f"Unknown simulator {simulator}")
        self.processes = SITL_START_FUNCTIONS[simulator]

    def start_simulator(self, build_dir=None):
        if self.simulator == "ArduPilot":
            [ardu_process, gz_process] = self.processes()
            gz_process.start_process()
            ardu_process.start_process()
        else:
            if build_dir:
                self.processes.set_cwd(cwd=build_dir)
            self.processes.start_process()

            print("Simulator Started")

    def stop_simulator(self):
        if self.simulator == "ArduPilot":
            [ardu_process, gz_process] = self.processes
            # Stop ArduPilot first...
            ardu_process.hard_stop_process()
            gz_process.hard_stop_process()
        else:
            self.processes.hard_stop_process()


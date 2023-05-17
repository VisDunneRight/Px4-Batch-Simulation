import sys

from subprocess_manager.subprocess_manager import SubProcessManager
import shlex
from typing import Union

def build_px4_command(px4_simulator):
    command = shlex.split(f"make px4_sitl {px4_simulator} HEADLESS=1")
    return SubProcessManager(px4_simulator.lower(), command=command)


def build_ardu_pilot_command(sim_speed=1, show_console=False, show_map=False):
    # Additional Sim Arguments
    print(sim_speed, show_console, show_map)
    console_cmd = "--console" if show_console else ""
    show_map_cmd = "--map" if show_map else ""

    # Commands for Ardu and Gazebo
    gz_command = shlex.split("gzserver --verbose worlds/iris_arducopter_runway.world")
    ardu_command = shlex.split(f"sim_vehicle.py -v ArduCopter -f gazebo-iris "
                               f"--speedup={sim_speed} "
                               f"{console_cmd} "
                               f"{show_map_cmd}"
                               )

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
        self.ardu_process = None
        self.gz_process = None
        self.simulator = simulator

        self.speed_factor = 1
        self.show_console = False
        self.show_map = False

        if simulator not in SITL_START_FUNCTIONS:
            raise ValueError(f"Unknown simulator {simulator}")

        self.processes = SITL_START_FUNCTIONS[simulator]

    def set_argument(self, argument: str, arg_setting: Union[int, bool]) -> None:
        if argument == "speed_factor" and not isinstance(arg_setting, int):
            print("NOT INT")
            return
        if argument in ["show_console", "show_map"] and not isinstance(arg_setting, bool):
            print("NOT BOOL")
            return
        setattr(self, argument, arg_setting)

    def start_simulator(self, build_dir=None):
        if self.simulator == "ArduPilot":
            [self.ardu_process, self.gz_process] = self.processes(self.speed_factor, self.show_console, self.show_map)
            self.gz_process.start_process()
            self.ardu_process.start_process()
        else:
            if build_dir:
                self.processes.set_cwd(cwd=build_dir)
            self.processes.start_process()
            print("Simulator Started")

    def stop_simulator(self):
        if self.simulator == "ArduPilot":
            # Stop ArduPilot first...
            self.ardu_process.hard_stop_process()
            self.gz_process.hard_stop_process()
        else:
            self.processes.hard_stop_process()


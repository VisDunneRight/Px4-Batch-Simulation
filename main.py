from simulator_manager.run_test_manager import RunTestManager
from simulator_manager.pre_flight_check import FlightPreCheck

from typing import NoReturn, Dict
import argparse
import json
import psutil
import sys
import os


def main() -> NoReturn:
    # Get user inputs
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--simulator", help="Simulation to use: Gazebo, JMavSim, ArduPilot or AirSim",
                        default="Gazebo")
    parser.add_argument("--log-dir",
                        help="Directory for log files", default="logs")
    parser.add_argument("--speed-factor", default=1,
                        help="how fast to run the simulation")
    parser.add_argument("--abort-early", action='store_true',
                        help="abort on first unsuccessful test")
    parser.add_argument("--verbose", default=False, action='store_true',
                        help="enable more verbose output")
    parser.add_argument("config_file", help="JSON config file to use")
    parser.add_argument("--build-dir", type=str,
                        default='px4Developer/Firmware/',
                        help="relative path where the px4 built files are stored")

    args = parser.parse_args()

    with open(args.config_file) as json_file:
        config = json.load(json_file)

    # Do preflight checks before running starting simulators and running missions.
    if not FlightPreCheck.is_ready(config, args):
        sys.exit(1)

    build_dir = None
    if args.simulator in ["Gazebo", "JMavSim"]:
        build_dir = args.build_dir

    run_test_manager = RunTestManager(config["tests"], args.simulator)
    run_test_manager.run_testcase(0, test_dir=config["test_directory"], build_dir=build_dir)


if __name__ == "__main__":
    main()

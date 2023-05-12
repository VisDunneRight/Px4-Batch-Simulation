import psutil
from typing import Dict
import os


class FlightPreCheck:
    @staticmethod
    def is_running(process_name: str) -> bool:
        for proc in psutil.process_iter(attrs=['name']):
            if proc.info['name'] == process_name:
                return True
        return False

    @staticmethod
    def is_ready(config: Dict[str, str], args: Dict[str, str]) -> bool:
        result = True

        if FlightPreCheck.is_running('px4'):
            print("px4 process already running\n"
                  "run `killall px4` and try again")
            result = False
        if args.simulator in ["Gazebo" "JMavSim"] and not os.path.isdir(args.build_dir):
            print("PX4 is not placed in build_dir\n"
                  "Make sure to install PX4 or correctly link to it before running this script "
                  )
            result = False
        if args.simulator == 'Gazebo':
            if FlightPreCheck.is_running('gzserver'):
                print("gzserver process already running\n"
                      "run `killall gzserver` and try again")
                result = False
            if FlightPreCheck.is_running('gzclient'):
                print("gzclient process already running\n"
                      "run `killall gzclient` and try again")
                result = False

        if not os.path.isdir(config['test_directory']):
            print("Cannot find directory containing tests\n"
                  "Update config file with correct location")
            result = False

        return result

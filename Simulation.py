#!/usr/bin/env python3

import json
import sys
import argparse
import psutil
import os
import datetime
import subprocess
import time
import threading
import atexit
from logger_helper import color, colorize
from typing import Any, Dict, List, NoReturn, Optional, TextIO

def main() -> NoReturn:
  parser = argparse.ArgumentParser()
  parser.add_argument("-s","--simulator", help="Simulation to use: Gazebo, JMavSim or AirSim",
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
  if not isEverythingReady(config, args):
    sys.exit(1)
  if args.verbose:
    print("Starting testing of missions.")
  tester = Tester(
        config,
        args.simulator,
        args.abort_early,
        args.speed_factor,
        args.log_dir,
        args.verbose,
        args.build_dir
    )

  sys.exit(0 if tester.run() else 1)


def isRunning(process_name: str) -> bool:
    for proc in psutil.process_iter(attrs=['name']):
        if proc.info['name'] == process_name:
            return True
    return False


def isEverythingReady(config: Dict[str, str], args: Dict[str,str]) -> bool:
    result = True

    if isRunning('px4'):
        print("px4 process already running\n"
              "run `killall px4` and try again")
        result = False
    if not os.path.isdir(args.build_dir):
        print("PX4 is not placed in build_dir\n"
              "Make sure to install PX4 or correctly link to it before running this script "
              )
        result = False
    if args.simulator == 'Gazebo':
        if isRunning('gzserver'):
            print("gzserver process already running\n"
                  "run `killall gzserver` and try again")
            result = False
        if isRunning('gzclient'):
            print("gzclient process already running\n"
                  "run `killall gzclient` and try again")
            result = False

    if not os.path.isdir(config['test_directory']):
        print("Cannot find directory containing tests\n"
              "Update config file with correct location")
        result = False

    return result


class Tester:
  def __init__(self,
                config: Dict[str, Any],
                simulator: str,
                abort_early: bool,
                speed_factor: float,
                log_dir: str,
                verbose: bool,
                build_dir: str):
    self.config = config
    self.build_dir = build_dir
    self.simulator = simulator
    # self.active_runners: List[ph.Runner]
    self.abort_early = abort_early
    self.tests = config['tests']
    self.speed_factor = speed_factor
    self.log_dir = log_dir
    self.verbose = verbose
    self.process = None
    self.start_time = datetime.datetime.now()
    self.log_fd: Any[TextIO] = None
    self.stop_thread: Any[threading.Event] = None

  def numCases(self) -> int:
    return len(self.tests)

  def run(self) -> bool:
    # self.show_plans()
    # self.prepare_for_results()
    self.runTests()
    # self.show_detailed_results()
    # self.show_overall_result()
    return True

  def runTests(self) -> None:
    for index, test in enumerate(self.tests):
      print("--> Test case {} of {}: '{}' running ...".
                      format(index + 1,
                             self.numCases(),
                             test['name']))
      # log_dir = self.get_log_dir(iteration, test['model'], key)
      # if self.verbose:
        # print("Creating log directory: {}".format(log_dir))
      # os.makedirs(log_dir, exist_ok=True)
      #TODO:Pass log_dir to test cases to save information
      was_success = 0
      for i in range(2):
        was_success = self.runTestCase(test)
        if was_success == 2:
          print("Rerun test case.")
          continue
        break

      print("--- Test case {} of {}: '{}' {}."
            .format(index + 1,
                    self.numCases(),
                    test['name'],
                    colorize("succeeded", color.GREEN)
                    if was_success == 0
                    else colorize("failed", color.RED)))

      if was_success != 0 and self.abort_early:
          print("Aborting early")
          return

    return
  def poll(self) -> Optional[int]:
        return self.process.poll()

  def process_output(self) -> None:
          assert self.process.stdout is not None
          while True:
              line = self.process.stdout.readline()
              if not line and \
                      (self.stop_thread.is_set() or self.poll is not None):
                  print("breaking process out")
                  break
              if not line or line == "\n":
                  continue
              # line = self.add_prefix(10, self.name, line)
              print(line,end='')
              # print(line, self.stop_thread.is_set(), self.poll, end='')


  #TODO: added check when case failed to run
  def runTestCase(self, test:Dict[str, Any]) -> bool:
    atexit.register(self.stopProcess)
    self.process = subprocess.Popen(
      [ "make", "HEADLESS=1", "px4_sitl", "gazebo"],
      cwd=self.build_dir,
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      universal_newlines=True
    )
    self.stop_thread = threading.Event()
    # self.thread = threading.Thread(target=self.process_output)
    # self.thread.start()
    
    time.sleep(10)
    try:
      missionCommand = ["python3", test['excutable']]
      if "command" in test:
        missionCommand.append(test['command'])
      mission = subprocess.run(missionCommand,
                              cwd=self.config['test_directory'],
                              timeout=160)
                              
      if(mission.returncode > 0):
        return 1
      else:
        return 0
    except subprocess.TimeoutExpired:
      print('Process ran too long')
    # self.process_output()
    self.stopProcess()
    return 2
    

  def stopProcess(self):
    #Kill all process and get control back
    atexit.unregister(self.stopProcess)
    if not self.stop_thread:
      return 0
    returnCode = self.process.poll()
    self.process.terminate()
    try:
      returnCode = self.process.wait(timeout=1)
    except:
      pass
    if returnCode is None:
      print("killing {}".format("Px4"))
      self.process.kill()
      returnCode = self.process.poll()
    #Gets control back to the shell
    self.stop_thread.set()
    print("Stop_thread set")
    # self.thread.join()
    os.system('stty sane') 
      
if __name__ == '__main__':
    main()
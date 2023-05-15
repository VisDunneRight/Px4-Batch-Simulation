from simulator_manager.simulator_manager import SimulatorManager
from subprocess import TimeoutExpired
import subprocess
import threading
import time
import shlex

TIME_BETWEEN_RUNS = 5
TIMEOUT_LENGTH = 1200


class RunTestManager:

    def __init__(self, tests, simulator):
        self.tests = tests
        self.simulator = simulator
        self.num_cases = len(tests)
        self.sim_manager = SimulatorManager(simulator)
        self.stop_thread = None

    def run_all_test(self):
        for index, test in enumerate(self.tests):
            print(f"---> Test case {index} of {self.num_cases}: running ... {test['name']}")

            was_success = 0
            # TODO: comeback to this...

    def run_testcase(self, test_number, test_dir, build_dir=None) -> int:
        test = self.tests[test_number]
        try:
            # Start the simulator
            self.sim_manager.start_simulator(build_dir)
            # self.stop_thread = threading.Event()
            print("Next run in {} seconds".format(TIME_BETWEEN_RUNS))

            # Run the mission
            mission_command = ["python3", test['executable']]

            if "command" in test:
                mission_command.append(test["command"])
                mission_command.append(self.simulator)

            print("running test")
            run_results = subprocess.run(
                mission_command,
                cwd=test_dir,
                timeout=TIMEOUT_LENGTH
            )

            print("RESULTS", run_results.returncode)

            self.sim_manager.stop_simulator()
            # TODO
            # self.stop_thread.stopProcess()
            if run_results.returncode > 0:
                return 1
            else:
                return 0

        except ValueError:
            print("The simulator " + self.simulator + "is not yet implemented.")
            self.sim_manager.stop_simulator()
            exit(1)
        except KeyError as error:
            print("KeyError:", error)

        except TimeoutExpired:
            print("Process ran to long...")

        self.sim_manager.stop_simulator()
        return 2

from simulator_manager.simulator_manager import SimulatorManager
from subprocess import TimeoutExpired
import subprocess
from logger_helper import color, colorize

TIME_BETWEEN_RUNS = 5
TIMEOUT_LENGTH = 900


class RunTestManager:

    def __init__(self, tests, args):
        self.tests = tests
        self.simulator = args.simulator
        self.num_cases = len(tests)
        self.sim_manager = SimulatorManager(self.simulator)
        self.stop_thread = None
        self.abort_early = args.abort_early
        self.save_log_dir = args.log_dir

        # Set adjust Simulators parameters.
        for key, value in vars(args).items():
            if key in ["show_console", "show_map", "speed_factor"]:
                self.sim_manager.set_argument(argument=key, arg_setting=value)

    def set_abort_early(self, abort_early=False):
        self.abort_early = abort_early

    def run_all_test(self, test_dir, build_dir=None) -> None:
        for index, test in enumerate(self.tests):
            print(f"---> Test case {index} of {self.num_cases}: running ... {test['name']}")

            was_success = 0
            for attempt_number in range(2):
                was_success = self.run_testcase(index, test_dir, build_dir)
                if was_success == 0:
                    break
                print(f"Re-running test case {index}")

            # Prints success or failure of the flight.
            success_message = colorize('succeeded', color.GREEN) if was_success == 0 else colorize("failed", color.RED)
            print(f"Test case {index + 1} of {self.num_cases}: {success_message}")

            if was_success != 0 and self.abort_early:
                print(colorize("ABORTING EARLY", color.YELLOW))
                return

    def run_testcase(self, test_number, test_dir, build_dir=None) -> int:
        test = self.tests[test_number]
        try:
            # Start the simulator
            self.sim_manager.start_simulator(build_dir)
            print("Next run in {} seconds".format(TIME_BETWEEN_RUNS))

            # Run the mission
            mission_command = ["python3", test['executable']]

            if "command" in test:
                mission_command.append(test["command"])
                mission_command.append(self.simulator)

            mission_command.append("--save-dir")
            mission_command.append(self.save_log_dir)

            # print("running test")
            run_results = subprocess.run(
                mission_command,
                cwd=test_dir,
                timeout=TIMEOUT_LENGTH
            )
            self.sim_manager.stop_simulator()
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

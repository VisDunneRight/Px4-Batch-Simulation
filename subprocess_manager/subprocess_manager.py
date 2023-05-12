import subprocess
import os
import psutil


class SubProcessManager:
    def __init__(self, process_name: str, command: list) -> None:
        self.command = command
        self.process = None
        self.name = process_name

    def start_process(self, cwd=None) -> None:
        if self.process is not None:
            raise RuntimeError(f"{self.name} is already running.")
        self.process = subprocess.Popen(
            self.command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=cwd,
            universal_newlines=True
        )

    def is_process_running(self):
        return self.process is not None and self.process.poll() is None

    def hard_stop_process(self):
        if self.process is None:
            raise RuntimeError(f"{self.name} is not running")
        parent = psutil.Process(self.process.pid)
        print(f"{self.name} process closing...")
        for child in parent.children(recursive=True):
            child.terminate()

        # Wait for all child processes to terminate
        gone, alive = psutil.wait_procs(parent.children(), timeout=5)

        # Kill any child processes that are still alive
        for child in alive:
            child.kill()

        print(f"{self.name} process stopped.")

    def stop_process(self):
        if self.process is None:
            raise RuntimeError(f"{self.name} is not running")
        self.process.terminate()
        print(f"{self.name} process closing...")
        self.process.wait()
        print(f"{self.name} process stopped.")


    def read_output(self):
        if self.process is None:
            raise RuntimeError(f"{self.name} is not running.")
        return self.process.stdout.read()

    def poll_process(self):
        return self.process.poll()

    @staticmethod
    def kill_process(process_ids):
        for pid in process_ids:
            os.kill(pid, 9)  # 9 is the signal number for SIGKILL

    @staticmethod
    def get_process_ids(process_name):
        return [int(pid) for pid in os.popen(f"pgrep {process_name}").read().split()]


import subprocess


class SubProcessManager:
    def __init__(self, process_name: str, command: list) -> None:
        self.command = command
        self.process = None
        self.name = process_name

    def start_process(self) -> None:
        if self.process is not None:
            raise RuntimeError(f"{self.name} is already running.")
        self.process = subprocess.Popen(
            self.command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            universal_newlines=True
        )

    def stop_process(self) -> None:
        if self.process is None:
            raise RuntimeError(f"{self.name} is not running")
        self.process.terminate()
        print(f"{self.name} closed")

    def is_process_running(self):
        return self.process is not None and self.process.poll() is None

    def wait_for_exit(self):
        if self.process is None:
            raise RuntimeError("Process is not running")
        self.process.wait()

    def read_output(self):
        if self.process is None:
            raise RuntimeError(f"{self.name} is not running.")
        return self.process.stdout.read()

    def poll_process(self):
        return self.process.poll()

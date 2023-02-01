import json
import os


class Batch:
    """ Helper to generate a batch mission plan"""

    def __init__(self, py_script: str,  mavlink_connection: str = "", test_directory: str = "missions/") -> None:

        self. py_script = py_script = py_script.strip(
            ".py") + ".py"  # ensures the py extension is there

        self.mission_plans = {
            "mavlink_connection": mavlink_connection,
            "test_directory": test_directory,
            "tests": []}

    def generate_batch_plan(self, batch_dir: str, has_sub_dir: False) -> None:
        mission_num = 1

        if not has_sub_dir:
            mission_names = [batch_dir.split(
                "/")[-2]] * (len(batch_dir.split("/")))
            tests = [os.path.join(batch_dir, ulog)
                     for ulog in os.listdir(batch_dir)]
        else:
            batch_names = os.listdir(batch_dir)
            batches = [os.path.join(batch_dir, mission)
                       for mission in batch_names]

            tests, mission_names = [], []
            for i, batch in enumerate(batches):
                logs = os.listdir(batch)
                for log in logs:
                    tests.append(os.path.join(batch, log))
                    mission_names.append(batch_names[i])

        for i, test in enumerate(tests):
            self.mission_plans["tests"].append(
                {
                    "name": f"Mission {mission_num}:  {mission_names[i]}",
                    "excutable": self.py_script,
                    "command": os.path.abspath(test),
                }
            )
            mission_num += 1

    def save_batch_plan(self, save_location: str, config_name: str) -> None:
        out_path = os.path.join(save_location, config_name)

        if len(self.mission_plans["tests"]) <= 0:
            raise Exception("There were no test added.")

        json_object = json.dumps(self.mission_plans, indent=4)

        with open(out_path, "w", encoding="utf8") as outfile:
            outfile.write(json_object)
            # json.dump(self.mission_plans, outfile)


def main():
    batch_run = Batch("sdk_ulog_mission.py")
    batch_run.generate_batch_plan(
        "/Users/daniellisko/GitRepos/Px4-Batch-Simulation/Px4_Simulations/lisko_getting_wp/saved_waypoints/waypoints/", True)

    batch_run.save_batch_plan(
        "/Users/daniellisko/GitRepos/Px4-Batch-Simulation/missions", "wp_configs.json")

    for item in batch_run.mission_plans["tests"]:
        print(item)


if __name__ == "__main__":
    main()

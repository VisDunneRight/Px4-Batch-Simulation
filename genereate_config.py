import argparse
import os
import json

config = {
    "mavlink_connection": "",
    "test_directory": "missions/",
    "tests": []
}


def create_config(dir_path, mission_path):
    files = os.listdir(dir_path)
    mission_path = mission_path.rstrip("/")

    files = [f for f in files if not f.startswith("._")]

    for i, file_name in enumerate(files):
        if file_name.endswith(".json"):
            pattern_name = file_name.split("_")[0]
            config["tests"].append(
                {
                    "name": f"Mission {i + 1}:  {pattern_name}",
                    "excutable": "sdk_ulog_mission.py",
                    "command": f"{mission_path}/{file_name}"
                }
            )

    print(f"Created {len(files)} missions.")
    return config


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument("dir_path", type=str, help="directory path to waypoints JSON files.")
    argparser.add_argument("output_path", type=str, default="./", help="where to save generated config file.")
    argparser.add_argument("mission_path", type=str, help="path to python script that will run the missions.")
    argparser.add_argument("-c", "--config_name",
                           type=str, default="config.json", help="name of the config file being generated.")

    args = argparser.parse_args()
    output_dict = create_config(args.dir_path, args.mission_path)

    with open(f"{args.output_path}/{args.config_name}", "w", encoding="UTF-8") as outfile:
        json.dump(output_dict, outfile, indent=4)

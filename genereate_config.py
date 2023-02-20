import argparse
import os
import json

config = {
    "mavlink_connection": "",
    "test_directory": "missions/",
    "tests": []
}


def create_config(dir_path):
    files = os.listdir(dir_path)

    print(len(files))
    for i, file_name in enumerate(files):
        if file_name.endswith(".json"):
            pattern_name = file_name.split("_")[0]
            config["tests"].append(
                {
                    "name": f"Mission {i + 1}:  {pattern_name}",
                    "excutable": "sdk_ulog_mission.py",
                    "command": f"/root/code/saved_wp/{file_name}"
                }
            )

    return config


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument("dir_path", type=str)
    argparser.add_argument("output_path", type=str, default="./")
    argparser.add_argument("-c", "--config_name",
                           type=str, default="config.json")

    args = argparser.parse_args()

    output_dict = create_config(args.dir_path)

    with open(f"{args.output_path}/{args.config_name}", "w", encoding="UTF-8") as outfile:
        json.dump(output_dict, outfile, indent=4)

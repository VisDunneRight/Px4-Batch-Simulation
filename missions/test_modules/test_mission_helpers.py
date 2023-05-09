import sys
sys.path.append("../")
from mission_helpers_module.arduino_mission_helpers.missionHelperDroneKit import Mission


def test_mission_arduino_dronekit():
    """RUN TEST"""
    mission = Mission()
    mission.connect()
    mission.arm()
    mission.takeoff(10)

    # Run the mission plan
    mission_plan = {
        "x": [50, 50, -50, -50],
        "y": [-50, 50, 50, -50],
        "altitude": [11, 12, 13, 14]
    }

    # Upload and start the mission.
    mission.upload_mission(mission_plan)
    mission.start_mission()
    mission.monitor_mission()

    # Finish Mission
    mission.return_to_launch()
    mission.close_connection()




if __name__ == "__main__":
    test_mission_arduino_dronekit()

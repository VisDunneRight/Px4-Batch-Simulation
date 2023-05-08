from constants import FlightMode, MessagingType
from pymavlink import mavutil
import math
from time import sleep
import sys


class Mission:
    def __init__(self) -> None:
        """ Mission Helper for ArduinoPilot"""
        self.sim_address = "udpin:127.0.0.1:14550"  # SITL connection
        self.vehicle = None
        self.waypoints = []

    def connect(self):
        """ Method for setting up communication links to MAVLink systems"""
        print("CONNECTING TO VEHICLE..")
        self.vehicle = mavutil.mavlink_connection(self.sim_address)
        self.vehicle.wait_heartbeat()
        print(
            f"Heartbeat from system (system: {self.vehicle.target_system}, component: {self.vehicle.target_component})")

        print(self.get_message())

    def arm(self):
        print("CHECKING FOR HEARTBEAT...")
        self.vehicle.wait_heartbeat()
        print("ARMING VEHICLE..")
        self.vehicle.arducopter_arm()

        # target_system = self.vehicle.target_system
        # target_component = self.vehicle.target_component
        # cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        # confirmation = 0
        # self.vehicle.mav.command_long_send(
        #     target_system, target_component, cmd, confirmation, 1, 0, 0, 0, 0, 0, 0)

        print("Arm command", self.get_message("COMMAND_ACK"), sep="\n")
        print("Waiting for arming...")
        self.vehicle.motors_armed_wait()
        print('Armed!')

    def disarm(self):

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)

        print("Disarm command", self.get_message("COMMAND_ACK"), sep="\n")

    def take_off(self):

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, math.nan, 0, 0, 10
        )

        msg = self.get_message("COMMAND_ACK").to_dict()
        print("Takeoff command", msg, sep="\n")

    def upload_mission(self, mission_items):
        n = len(mission_items)
        print("Sending", n, "mission items")

        self.vehicle.mav.mission_count_send(
            self.vehicle.target_system, self.vehicle.target_component, n, 0)

        print(self.get_message(MessagingType.MISSION_REQUEST), sep="\n")
        for waypoint in mission_items:
            self.vehicle.mav.mission_item_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                waypoint.seq,  # Sequence
                waypoint.frame,  # Frame
                waypoint.command,  # Command
                waypoint.current,  # Current
                waypoint.auto_continue,  # Auto continue
                waypoint.hold_time,
                waypoint.accepted_radius,
                waypoint.mission_type,
                waypoint.yaw,
                waypoint.x,
                waypoint.y,
                waypoint.z,
                waypoint.mission_type
            )

            if waypoint != mission_items[-1]:
                msg = self.get_message(
                    MessagingType.MISSION_REQUEST)
                print(msg)

        upload_results = self.get_message(MessagingType.MISSION_ACK).to_dict()

        if upload_results["type"] == 0:
            print("Mission upload successful")
        else:
            print("Mission upload failed")
            # TODO:handle mission upload failure

    def start_mission(self):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        print(self.get_message(MessagingType.MISSION_ACK), sep="\n")

    def change_flight_mode(self, flight_mode: str) -> None:
        if flight_mode not in self.vehicle.mode_mapping():
            print("Unknown flight mode", flight_mode)
            print("Try: ", self.vehicle.mode_mapping())
            sys.exit(1)

        mode_id = self.vehicle.mode_mapping()[flight_mode]
        self.vehicle.set_mode(mode_id)

        while True:
            ack_msg = self.get_message(MessagingType.COMMAND_ACK).to_dict()
            if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                sleep(0.5)
                continue
            print(
                mavutil.mavlink.enums[MessagingType.MAV_RESULT][ack_msg['result']].description)
            break

    def set_home(self, latitude, longitude, alt) -> None:
        print("Setting home")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            0, 0, 0, 0, math.nan, latitude, longitude, alt)

        print(self.get_message("COMMAND_ACK"), sep="\n")

    def return_home(self):
        pass

    def get_message(self, keyword=None):
        if keyword:
            return self.vehicle.recv_match(type=keyword, blocking=True)

        return self.vehicle.recv_match(blocking=True)


class MissionItem:
    """_summary_
    For adding mission items to a flight plan.
    """
    def __init__(self, i, current, x, y, z) -> None:
        self.seq = i
        # Allows us to tell where are drone is in relation to the earth
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT  # Move to waypoint
        self.current = current
        self.auto_continue = 1  # Tells drone to continue to the next waypoint
        self.hold_time = 0.0
        self.accepted_radius = 2.0  # Considered reached wp within 2 meters.
        self.pass_radius = 20.00
        self.yaw = math.nan  # We want it to face next waypoint
        self.x = x
        self.y = y
        self.z = z
        self.mission_type = 0


if __name__ == "__main__":
    print("RUNNING Mission")
    mission = Mission()
    mission.connect()
    print("Connected to Vehicle")

    mission.upload_mission(
        [
            MissionItem(0, 0, 0, 0, 0),
            MissionItem(1, 0, 0, 0, 0),
            MissionItem(2, 0, 0, 0, 0),
            MissionItem(3, 0, 0, 0, 0)
        ]
    )

    print("CHANGE FLIGHT MODE")
    mission.change_flight_mode(FlightMode.GUIDED)
    sleep(5)
    mission.arm()
    mission.take_off()

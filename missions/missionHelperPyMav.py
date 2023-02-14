from pymavlink import mavutil
from time import sleep
import math


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

        print(self.get_message().to_dict())

    def arm(self):
        target_system = self.vehicle.target_system
        target_component = self.vehicle.target_component
        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        confirmation = 0
        self.vehicle.mav.command_long_send(
            target_system, target_component, cmd, confirmation, 1, 0, 0, 0, 0, 0, 0)

        print("Arm command", self.get_message("COMMAND_ACK"), sep="\n")
        print("Waiting for the vehicle to arm")
        self.vehicle.motors_armed_wait()
        print('Armed!')

    def disarm(self):
        target_system = self.vehicle.target_system
        target_component = self.vehicle.target_component
        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        confirmation = 0

        self.vehicle.mav.command_long_send(
            target_system, target_component, cmd, confirmation, 0, 0, 0, 0, 0, 0, 0)

        print("Disarm command", self.get_message("COMMAND_ACK"), sep="\n")

    def take_off(self):

        target_system = self.vehicle.target_system
        target_component = self.vehicle.target_component
        cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        confirmation = 0

        self.vehicle.mav.command_long_send(
            target_system, target_component, cmd, confirmation, math.nan, 0, 0, 0, 0, 0, 10)

        msg = self.get_message("COMMAND_ACK").to_dict()
        print("Takeoff command", msg, sep="\n")

    def upload_mission(self, mission_items):
        n = len(mission_items)
        print("Sending", n, "mission items")

        self.vehicle.mav.mission_count(
            self.vehicle.target_system, self.vehicle.target_component, n, 0)
        print(self.get_message("COUNT_ACK"), sep="\n")

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
                waypoint.mission_type
            )

    def return_home(self):
        pass

    def get_message(self, keyword=None):
        if keyword:
            return self.vehicle.recv_match(type=keyword, blocking=True)

        return self.vehicle.recv_match(blocking=True)

    def print_status(self):
        while True:
            print(self.get_message("GLOBAL_POSITION_INT").to_dict())


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
    mission.arm()

    mission.upload_mission([
        MissionItem(1, 0, 0, 0, 0),
        MissionItem(2, 0, 0, 0, 0),
        MissionItem(3, 0, 0, 0, 0),
        MissionItem(4, 0, 0, 0, 0)]
    )

    # print("Taking off")
    # mission.take_off()

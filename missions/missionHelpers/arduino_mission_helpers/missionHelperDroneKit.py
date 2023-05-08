import sys
import time
from dronekit import connect, VehicleMode, Command
from constants import RTL, GUIDED, AUTO
from pymavlink import mavutil
from missionHelpers.abstract_modules.abstract_mission_helper import AbstractMissionHelper


class Mission(AbstractMissionHelper):
    """ Mission Helper for ArduinoPilot"""

    def __init__(self, connection_string="127.0.0.1:14551"):
        self.sim_address = connection_string
        self.vehicle = None
        self.waypoints = []
        self.home_location = None

    def get_home_location(self) -> tuple:
        """ Get the starting location for the vehicle."""
        return self.home_location

    def set_home_location(self):
        try:
            self.home_location = self.vehicle.location.global_frame
        except AttributeError as error:
            print("Vehicle wasn't set...")
            sys.exit(1)  # General Error

    def connect(self):
        """Connect to the vehicle."""
        print(f"Connecting to {self.sim_address}")
        self.vehicle = connect(self.sim_address)
        self.set_home_location()
        print("HOME LAT", self.home_location.lat, "HOME LON", self.home_location.lon)

    def arm(self):
        """ Arm the vehicle"""
        print("Waiting to arm...")
        while not self.vehicle.is_armable:
            time.sleep(1)
        self.vehicle.mode = VehicleMode(GUIDED)
        self.vehicle.armed = True
        while not self.vehicle.arm:
            time.sleep(1)

    def add_mission_item(self, d_north, d_east, altitude, cmd_type):
        """Add a new waypoint to the mission"""
        waypoint = self.get_offset_location(self.home_location, d_north, d_east, )
        cmd = Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            cmd_type,
            0,
            0,
            0,
            0,
            0,
            0,
            waypoint.lat,
            waypoint.lon,
            altitude
        )
        self.vehicle.commands.add(cmd)

    def upload_mission(self, waypoints):
        """ Upload a list of waypoints from"""
        self.vehicle.commands.clear()
        print("Adding waypoints...")
        # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        self.add_mission_item(0, 0, 10, cmd_type=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        for x, y, altitude in zip(waypoints["x"], waypoints["y"], waypoints["altitude"]):
            self.add_mission_item(d_north=y, d_east=x, altitude=altitude, cmd_type=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)

        # Add dummy waypoint to let us know when we've reached destination.
        self.add_mission_item(waypoints["x"][-1], waypoints["y"][-1], waypoints["altitude"][-1],
                              cmd_type=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
        print("Uploading Mission")
        self.vehicle.commands.upload()

    def takeoff(self, altitude):
        """ Take of to input altitude"""
        print("Taking off to altitude: ", altitude)
        self.vehicle.simple_takeoff(altitude)

        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            if current_altitude >= altitude * 0.95:
                print("Target altitude reached")
                break
            time.sleep(1)

    def start_mission(self):
        """Start the mission."""
        self.vehicle.commands.next = 0

        # This will start the mission
        self.vehicle.mode = VehicleMode(AUTO)

    def monitor_mission(self):
        """Monitor the mission."""
        total_waypoints = self.vehicle.commands.count
        while True:
            next_waypoint = self.vehicle.commands.next
            print(f"Mission progress: "
                  f"{next_waypoint}/"
                  f"{total_waypoints}", end="\r")
            # if next_waypoint + 2  == self.vehicle.commands.count:
            #     print("Skip to last waypoint")
            if next_waypoint == self.vehicle.commands.count:
                break
            time.sleep(1)

    def return_to_launch(self):
        """Return to home location"""
        print("Returning to home location")
        self.vehicle.mode = VehicleMode(RTL)

    def close_connection(self):
        """Close vehicle connection"""
        # self.vehicle.download()
        print("Closing connection")
        self.vehicle.close()


def main():
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


if __name__ == '__main__':
    main()

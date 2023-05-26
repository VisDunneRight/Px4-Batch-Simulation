import shutil
import sys
import os
import time
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
from mission_helpers_module.arduino_mission_helpers.constants import RTL, GUIDED, AUTO
from ..abstract_modules.abstract_mission_helper import AbstractMissionHelper

TAKEOFF_ALT = 10


class Mission(AbstractMissionHelper):
    """Mission Helper for ArduinoPilot.

    This class provides helper methods to manage missions for the ArduinoPilot.
    It inherits from the AbstractMissionHelper class.
    """

    def __init__(self, connection_string="127.0.0.1:14550"):
        self.air_speed = None  # Use to check if we've already set the vehicles AirSpeed.
        self.sim_address = connection_string
        self.vehicle = None
        self.waypoints = []
        self.mission_items = []
        self.home_location = None
        self.download_dir = "../logs"
        self.logs_dir = "../logs"
        self.mission_name = None

    async def get_home_location(self) -> tuple:
        """Get the starting location for the vehicle.

        Returns: tuple: A tuple containing the latitude and longitude of the starting location.
        """
        return self.home_location.lat, self.home_location.lon

    def set_mission_name(self, mission_name):
        """Set the name of the mission.
        Args: mission_name (str): The name of the mission.
        """
        self.mission_name = mission_name

    def set_log_dir(self, download_dir):
        """Set the directory for downloading logs.

        Args: download_dir (str): The directory for downloading logs.
        """
        self.download_dir = download_dir

    def set_home_location(self):
        """Connect to the vehicle."""
        try:
            self.home_location = self.vehicle.location.global_frame
        except AttributeError as error:
            print(error)
            print("Vehicle wasn't set...")
            sys.exit(1)  # General Error

    async def connect(self):
        """Connect to the vehicle."""
        print(f"Connecting to {self.sim_address}")
        self.vehicle = connect(self.sim_address)
        self.vehicle.wait_for_armable()
        print(self.vehicle)
        self.set_home_location()
        print("HOME LAT", self.home_location.lat, "HOME LON", self.home_location.lon)
        await self.clear_flight_logs()
        print(os.listdir(os.curdir))

    async def arm(self):
        """ Arm the vehicle"""
        print("Waiting to arm...")
        while not self.vehicle.is_armable:
            time.sleep(1)
        self.vehicle.mode = VehicleMode(GUIDED)
        self.vehicle.armed = True
        while not self.vehicle.arm:
            time.sleep(1)

    def add_mission_item(self,
                         latitude,
                         longitude,
                         altitude,
                         cmd_type=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         speed=None):
        """Add a new waypoint to the mission.

        This function creates a new waypoint based on the provided parameters and adds it to the mission.
        The waypoint is created as an offset from the home location.

        Args:
            latitude (float): The latitude of the waypoint.
            longitude (float): The longitude of the waypoint.
            altitude (float): The altitude of the waypoint.
            cmd_type (int): The command type for the waypoint (default: mavutil.mavlink.MAV_CMD_NAV_WAYPOINT).
            speed (float): The speed of the vehicle (default: None).

        Note:
            The parameters 'd_north' and 'd_east' represent displacements in the northward (x) and eastward (y)
            directions respectively, relative to the home location. These are not equivalent to latitude and longitude
            but are converted to such by the 'get_offset_location' function.
        """

        # Sets a standard AirSpeed if it hasn't been set
        if self.air_speed is None:
            self.air_speed = speed
        self.mission_items.append(self.generate_mission_item(latitude, longitude, altitude, cmd_type))

    @staticmethod
    def generate_mission_item(latitude, longitude, altitude, cmd_type=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
        """Generate a mission item.

       Args:
           latitude (float): The latitude of the mission item.
           longitude (float): The longitude of the mission item.
           altitude (float): The altitude of the mission item.
           cmd_type (int): The command type for the mission item (default: mavutil.mavlink.MAV_CMD_NAV_WAYPOINT).

       Returns:
           Command: The generated mission item command.
        """
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
            latitude,
            longitude,
            altitude
        )
        return cmd

    async def clear_mission(self):
        """Clear the mission."""
        self.vehicle.commands.clear()

    async def upload_mission(self):
        """Upload the mission to the vehicle."""
        # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        takeoff_item = self.generate_mission_item(0, 0, TAKEOFF_ALT, cmd_type=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        self.set_drone_speed()
        self.mission_items = [takeoff_item] + self.mission_items
        # Add dummy waypoint to let us know when we've reached destination.
        self.mission_items.append(self.mission_items[-1])

        # Upload Mission to Vehicle
        for mission_item in self.mission_items:
            self.vehicle.commands.add(mission_item)
        print("Uploading Mission to Vehicle...")
        self.vehicle.commands.upload()

    def set_drone_speed(self):
        """Set the drone's speed."""
        if self.air_speed is None:
            raise ValueError("AirSpeed Not set")
        print("SPEED", self.air_speed)
        self.vehicle.parameters["WPNAV_SPEED"] = self.air_speed * 100

    def takeoff(self, altitude):
        """Take off to the specified altitude.
        Args:
            altitude (float): The target altitude for takeoff.
        """
        print("Taking off to altitude: ", altitude)
        self.vehicle.simple_takeoff(altitude)

        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            if current_altitude >= altitude * 0.95:
                print("Target altitude reached")
                break
            time.sleep(1)

    async def start_mission(self):
        """Start the mission."""
        self.vehicle.commands.next = 0
        self.takeoff(10)
        # This will start the mission
        self.vehicle.mode = VehicleMode(AUTO)
        await self.monitor_mission()

    async def monitor_mission(self):
        """Monitor the mission."""
        total_waypoints = self.vehicle.commands.count
        while True:
            next_waypoint = self.vehicle.commands.next
            print(f"Mission progress: "
                  f"{next_waypoint}/"
                  f"{total_waypoints - 1}", end="\r")
            if next_waypoint == self.vehicle.commands.count:
                break
            time.sleep(1)

    async def return_to_launch(self):
        """Return to home location"""
        print("Returning to home location")
        self.vehicle.mode = VehicleMode(RTL)
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            if current_altitude >= 1:
                print("Landed")
                break
            time.sleep(1)

    async def close_connection(self):
        """Close vehicle connection"""
        await self.return_to_launch()
        await self.download_flight_log()
        print("Closing connection")
        self.vehicle.close()

    async def download_flight_log(self):
        """Download the flight log files."""
        files = os.listdir(self.logs_dir)
        os.makedirs(self.download_dir, exist_ok=True)
        for file in files:
            if ".bin" in file.lower():
                new_name = f"sim_{self.mission_name}.bin"
                src = os.path.join(self.logs_dir, file)
                dest = os.path.join(self.download_dir, new_name)
                os.rename(src, os.path.join(self.logs_dir, new_name))
                shutil.move(os.path.join(self.logs_dir, new_name), dest)

        await self.clear_flight_logs()

    async def clear_flight_logs(self):
        """Clear the flight log files."""
        if not os.path.isdir(self.logs_dir):
            os.makedirs(self.logs_dir)

        items_in_dir = os.listdir(self.logs_dir)
        if len(items_in_dir) > 0:
            for file in items_in_dir:
                if not file.lower().startswith("sim"):
                    file_path = os.path.join(self.logs_dir, file)
                    os.remove(file_path)

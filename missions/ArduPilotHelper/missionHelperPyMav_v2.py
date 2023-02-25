import dronekit_sitl
from pymavlink import mavutil
from dronekit import connect, VehicleMode, APIException, Command, LocationGlobalRelative, LocationGlobal
import socket
import time
import json
import math
import argparse
from time import sleep

sitl = dronekit_sitl.start_default()


class Mission:
    """Mission class for controlling the drone"""

    def __init__(self):
        self.connection_string = sitl.connection_string()
        self.vehicle = None
        self.mission_cmds = None
        self.num_missions = 0

    def connect(self):
        """Connect to the vehicle"""
        print(f"{self.connection_string}")
        print("Connecting to vehicle...")
        try:
            self.vehicle = connect(
                self.connection_string, wait_ready=True)
            print("CONNECTED...")

        except socket.error as e:
            print("No server exists at {}".format(self.connection_string))
        except OSError as e:
            print("No serial exist")
        except APIException as e:
            print("Timeout")
        except:
            print("Unexpected error:", sys.exc_info()[0])

    def arm(self):
        """Arms the vehicle"""
        # Don't let the vehicle arm until autopilot is ready
        out_msg = "Waiting for vehicle to be armable."
        while not self.vehicle.is_armable:
            print(out_msg, end="\r")
            out_msg += "."
            time.sleep(1)

        print("Arming vehicle...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        out_msg = "Waiting for vehicle to arm."
        while not self.vehicle.armed:
            print(out_msg, end="\r")
            out_msg += "."
            time.sleep(1)
        print("-----VEHICLE ARMED------")

    def takeoff(self, target_altitude=None):
        self.vehicle.simple_takeoff(target_altitude)

        if target_altitude is not None:
            print("Taking off to target altitude...")
            # triggers just below target altitude
            while not self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                time.sleep(1)
            print("Target altitude reached")

    def get_location_offset_meters(self, original_location, dNorth, dEast, alt):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon, original_location.alt + alt)

    def load_mission(self, uav_data):
        """Loads a mission"""

        cmds = self.vehicle.commands
        cmds.clear()
        home = self.vehicle.location.global_relative_frame

        print("Generating mission commands...")
        x_coord = uav_data["x"]
        y_coord = uav_data["y"]
        alt = uav_data["altitude"]
        s = uav_data["velocity"]

        i = 0
        for x, y, missionAlt, missionSpd in zip(x_coord, y_coord, alt, s):

            wp = self.get_location_offset_meters(home, y, x, missionAlt)
            cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
            cmds.add(cmd)

        return cmds

    def track_position(self):
        """Starts the mission"""
       # monitor mission execution
        self.vehicle.commands.next = 0
        nextwaypoint = self.vehicle.commands.next
        while nextwaypoint < len(self.vehicle.commands):
            if self.vehicle.commands.next > nextwaypoint:
                display_seq = self.vehicle.commands.next
                print("Moving to waypoint %s" % display_seq)
                nextwaypoint = self.vehicle.commands.next
            time.sleep(1)

    def get_home_location(self):
        uav_loc = self.vehicle.location.global_relative_frame
        print("HOME Location:", "Lat:", uav_loc.lat, "Long:", uav_loc.lon)
        return uav_loc.lat, uav_loc.lon

    def close_simulation(self):
        """Close the simulation"""
        print("Disconnecting from vehicle...")
        self.vehicle.close()
        print("Stopping SITL...")
        sitl.stop()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("mission_path", type=str,
                        help="the path to the flight plan")
    # parser.add_argument("ulog_save_path", type=str,
    #                     help="the path to save the ulog file")

    args = parser.parse_args()

    with open(args.mission_path, "r", encoding="utf-8") as input_data:
        uav_data = json.load(input_data)

    mission = Mission()
    mission.connect()

    homeLat, homeLon = mission.get_home_location()
    print(f'home location\n\t>lat:{homeLat}\n\t>lon:{homeLon}')

    cmds = mission.load_mission(uav_data)
    sleep(2)
    print("--Upload mission")
    cmds.upload()
    sleep(2)
    mission.arm()
    mission.takeoff(10)

    mission.vehicle.mode = VehicleMode("AUTO")
    # monitor mission execution
    mission.track_position()

    # while mission.vehicle.commands.next > 0:
    #     time.sleep(1)
    #     print(mission.vehicle.commands.next)

    mission.close_simulation()

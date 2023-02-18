import dronekit_sitl
from pymavlink import mavutil
from dronekit import connect, VehicleMode, APIException, Command, LocationGlobalRelative, LocationGlobal
import socket
import time

sitl = dronekit_sitl.start_default()


class Mission:
    """Mission class for controlling the drone"""

    def __init__(self):
        self.connection_string = sitl.connection_string()
        self.vehicle = None
        self.mission_cmds = None

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
        print("Running basic prearm checks...")

        # Don't let the vehicle arm until autopilot is ready

        while not self.vehicle.is_armable:
            print("Waiting for vehicle to be armable...")
            time.sleep(1)
        print("Arming vehicle...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for vehicle to arm...")
            time.sleep(1)

    def takeoff(self, target_altitude=None):
        self.vehicle.simple_takeoff(target_altitude)

        if target_altitude is not None:
            print("Taking off to target altitude...")
            # triggers just below target altitude
            while not self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                time.sleep(1)
            print("Target altitude reached")

    def load_mission(self, missions_array):
        """Loads a mission"""

        self.mission_cmds = self.vehicle.commands
        print("Clearing any existing missions...")
        self.mission_cmds.clear()

        print("Loading mission...")
        # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        self.mission_cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

        # Define  MAV_CMD_NAV_WAYPOINT locations and add the commands
        for mission in missions_array:
            break

    def get_home_location(self):
        uav_loc = self.vehicle.location.global_relative_frame
        print("DRONE---->", uav_loc.lat, uav_loc.lon)

    def close_simulation(self):
        """Close the simulation"""
        print("Disconnecting from vehicle...")
        self.vehicle.close()
        print("Stopping SITL...")
        sitl.stop()


if __name__ == "__main__":
    mission = Mission()
    mission.connect()
    mission.arm()
    print(mission.vehicle.mode)
    mission.get_home_location()
    mission.takeoff(20)
    mission.close_simulation()

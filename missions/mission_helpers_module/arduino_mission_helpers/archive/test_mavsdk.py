from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math

# Connect to the vehicle
connection_string = "127.0.0.1:14550"  # Change this to your connection string
print(f"Connecting to vehicle on: {connection_string}")
vehicle = connect(connection_string)

# Function to arm and takeoff to specified altitude
def arm_and_takeoff(altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {altitude} meters")
    vehicle.simple_takeoff(altitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.1f} m")
        if current_altitude >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_location_metres(original_location, d_north, d_east):
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))

    new_lat = original_location.lat + (d_lat * 180.0 / math.pi)
    new_lon = original_location.lon + (d_lon * 180.0 / math.pi)

    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)


def add_square_mission(vehicle, side_length=50):
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    start_location = vehicle.location.global_relative_frame
    altitude = start_location.alt

    waypoints = [
        get_location_metres(start_location, side_length, 0),
        get_location_metres(start_location, side_length, side_length),
        get_location_metres(start_location, 0, side_length),
        get_location_metres(start_location, 0, 0),
    ]

    cmds.clear()
    for waypoint in waypoints:
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, altitude)
        cmds.add(cmd)
    cmds.upload()



# Takeoff to 10 meters
target_altitude = 10
arm_and_takeoff(target_altitude)

# Add a square mission and start it
print("Adding square mission")
add_square_mission(vehicle, side_length=200)
print("Starting mission")
vehicle.mode = VehicleMode("AUTO")

# Wait 5 seconds
print("Hold for 5 seconds")
time.sleep(5)

print("RUNNING MISSION!!!")
while True:
    print("wp", vehicle.commands.next)
    if vehicle.commands.next == 3:
        break

# Land the vehicle
print("Landing...")
vehicle.mode = VehicleMode("LAND")

# Wait for the vehicle to land
while vehicle.armed:
    print("Waiting for landing...")
    time.sleep(1)

# Disconnect from the vehicle
print("Disconnecting...")
vehicle.close()
print("Done!")

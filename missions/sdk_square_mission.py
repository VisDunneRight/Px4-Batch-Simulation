import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import math


async def connect_sitl():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    return drone

async def arm_drone(drone):
    await drone.action.arm()

async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode is not previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")

async def takeoff(drone, takeoff_alt):
    # await drone.action.set_takeoff_altitude(altitude=takeoff_alt)     # Has issues
    await drone.param.set_float_param("MIS_TAKEOFF_ALT", takeoff_alt)
    await drone.action.takeoff()
    print('taking off..')
    prev_alt = None
    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)
        if altitude != prev_alt:
            prev_alt = altitude
            print(f'altitude: {altitude}')
            if altitude == round(takeoff_alt):
                print('takeoff_alt reached successfully!')
                break
    return drone

async def land(drone=None):
    '''
    Drone is expected to be passed as an input if the function is called within a mission function (drone already connected)
    otherwise, drone is None and the function will connect to it.
    '''
    if drone is None:
        drone = await connect_sitl()
    await drone.action.land()
    print('landing..')
    return drone

async def takeoff_land(takeoff_alt):
    drone = await connect_sitl()
    asyncio.ensure_future(print_flight_mode(drone)) # Start parallel task
    await arm_drone(drone)

    drone = await takeoff(drone=drone, takeoff_alt=takeoff_alt)
    # await asyncio.sleep(10)
    await land(drone)

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")

# used as a termination task in missions, so that the script keeps running as long as drone is in air
async def observe_is_in_air(drone):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            await asyncio.get_event_loop().shutdown_asyncgens()
            return

async def get_home_lat_lon(drone):
    async for position in drone.telemetry.position():
        home_lat = position.latitude_deg
        home_lon = position.longitude_deg
        break
    return home_lat, home_lon


def get_location_offset_meters(original_lat, original_lon, dNorth, dEast):
    earth_radius = 6378137.0    # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_lat/180))

    #New position in decimal degrees
    newlat = original_lat + (dLat * 180/math.pi)
    newlon = original_lon + (dLon * 180/math.pi)
    return newlat, newlon

async def square_mission(mission_alt=20, mission_spd=10, mission_north=10, mission_east=10, mission_south=-10):
    drone = await connect_sitl()
    # asyncio.ensure_future(print_mission_progress(drone)) # Parallel task
    termination_task = asyncio.ensure_future(observe_is_in_air(drone)) # keeps script running if drone in air

    home_lat, home_lon = await get_home_lat_lon(drone)
    print(f'home location\n\t>lat:{home_lat}\n\t>lon:{home_lon}')
    await drone.mission.clear_mission()     # Clear previous missions stored on drone
    mission_items = []
    # Takeoff
    mission_items.append(MissionItem(home_lat,
                                     home_lon,
                                     mission_alt,
                                     mission_spd,
                                     is_fly_through=True,
                                     gimbal_pitch_deg=0,
                                     gimbal_yaw_deg=0,
                                     camera_action=MissionItem.CameraAction.NONE,
                                     loiter_time_s=float('nan'),
                                     camera_photo_interval_s=float('nan'),
                                     acceptance_radius_m=float('nan'),
                                     yaw_deg=float('nan'),
                                     camera_photo_distance_m=float('nan')))
    # Setting Mission waypoints
    wp_north = get_location_offset_meters(home_lat, home_lon, dNorth=mission_north, dEast=0)
    wp_east = get_location_offset_meters(wp_north[0], wp_north[1], dNorth=0, dEast=mission_east)
    wp_south = get_location_offset_meters(wp_east[0], wp_east[1], dNorth=mission_south, dEast=0)
    # Setting mission waypoints
    mission_items.append(MissionItem(wp_north[0],
                                     wp_north[1],
                                     mission_alt,
                                     mission_spd,
                                     is_fly_through=True,
                                     gimbal_pitch_deg=0,
                                     gimbal_yaw_deg=0,
                                     camera_action=MissionItem.CameraAction.NONE,
                                     loiter_time_s=float('nan'),
                                     camera_photo_interval_s=float('nan'),
                                     acceptance_radius_m=float('nan'),
                                     yaw_deg=float('nan'),
                                     camera_photo_distance_m=float('nan')))

    mission_items.append(MissionItem(wp_east[0],
                                     wp_east[1],
                                     mission_alt,
                                     mission_spd,
                                     is_fly_through=True,
                                     gimbal_pitch_deg=0,
                                     gimbal_yaw_deg=0,
                                     camera_action=MissionItem.CameraAction.NONE,
                                     loiter_time_s=float('nan'),
                                     camera_photo_interval_s=float('nan'),
                                     acceptance_radius_m=float('nan'),
                                     yaw_deg=float('nan'),
                                     camera_photo_distance_m=float('nan')))

    mission_items.append(MissionItem(wp_south[0],
                                     wp_south[1],
                                     mission_alt,
                                     mission_spd,
                                     is_fly_through=True,
                                     gimbal_pitch_deg=0,
                                     gimbal_yaw_deg=0,
                                     camera_action=MissionItem.CameraAction.NONE,
                                     loiter_time_s=float('nan'),
                                     camera_photo_interval_s=float('nan'),
                                     acceptance_radius_m=float('nan'),
                                     yaw_deg=float('nan'),
                                     camera_photo_distance_m=float('nan')))
    
    await drone.mission.set_return_to_launch_after_mission(True)    # RTL after last wp
    mission_plan = MissionPlan(mission_items)
    await drone.mission.upload_mission(mission_plan)
    await drone.action.arm()
    await drone.mission.start_mission()
    await termination_task

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    # loop.run_until_complete(takeoff_land(20))
    loop.run_until_complete(square_mission(mission_alt=20, mission_spd=30, 
                                                           mission_north=10, 
                                                           mission_east=10, 
                                                           mission_south=-10))

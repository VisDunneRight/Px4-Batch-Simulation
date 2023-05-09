from abc import ABC, abstractmethod
import math

try:
    from dronekit import LocationGlobal
except ModuleNotFoundError:
    class LocationGlobal:
        def __init__(self):
            self.lat = None
            self.lon = None
            self.alt = None


class AbstractMissionHelper(ABC):
    """
    Abstract base class for mission helpers.

    This class defines the interface for mission helpers, which are used to help
    automate the process of arming, connecting, and uploading missions to drones.

    To create a concrete mission helper class, inherit from this class and
    implement all of its abstract methods.

    Methods:
        arm(): Abstract method that arms the drone.
        connect() -> None: Abstract method that connects to the drone.
        start_mission() -> None: Abstract method that starts a mission on the drone.
        add_mission_item(mission_item: Any) -> None: Abstract method that adds a mission item to the mission.
        upload_mission(mission_items: List[Any]) -> None: Abstract method that uploads the mission to the drone.
        get_home_location() -> Tuple[float, float, float]: Abstract method that gets the home location of the drone.
    """

    @abstractmethod
    def arm(self):
        """Arm the drone.

        This method should arm the drone so that it is ready to execute the mission.

        Returns:
            None
        """
        pass

    @abstractmethod
    def connect(self) -> None:
        """Connect to the drone.

        This method should establish a connection to the drone so that the mission
        can be uploaded and executed.

        Returns:
            None
        """
        pass

    @abstractmethod
    def start_mission(self) -> None:
        """Start a mission on the drone.

        This method should start the mission on the drone so that it begins executing
        the sequence of mission items.

        Returns:
            None

        Note:
            This function assumes you've added all mission items and that you've uploaded them.
        """
        pass

    @abstractmethod
    def add_mission_item(self, mission_item):
        """Add a mission item to the mission.

        This method should add a mission item to the mission sequence.

        Args:
            mission_item (Any): A mission item to add to the mission.

        Returns:
            None
        """
        pass

    @abstractmethod
    def upload_mission(self, mission_items) -> None:
        """Upload the mission to the drone.

        This method should upload the mission to the drone so that it can be executed.

        Args:
            mission_items (List[Any]): A list of mission items to upload to the drone.

        Returns:
            None
        """
        pass

    @abstractmethod
    def get_home_location(self) -> tuple:
        """
        Get the home location of the drone.

        This method should get the home location of the drone.

        Returns:
            A tuple containing the latitude, longitude, and altitude of the home location.
        """
        pass

    @abstractmethod
    def monitor_mission(self):
        """
        Monitor the progress of the current mission and take appropriate actions based on the status.

        This method continuously checks the status of the current mission and takes appropriate actions based on the
        status. For example, if the mission is complete, the method may return the vehicle to the home location or
        switch to a new mission. If there is a problem with the mission, the method may abort the mission and return
        the vehicle to the home location. :rtype: None
        """
        pass

    @abstractmethod
    def set_home_location(self):
        """
        Set the home location of the drone.

        This method will set the home location of the drone.

        Returns:
           A tuple containing the latitude, longitude, and altitude of the home location.
        """
        pass

    @staticmethod
    def get_offset_location(original_location, d_north, d_east):
        """
        Calculate a new location based on the original location and the specified north and east displacements.

        The northward displacement (d_north) represents a positive change in latitude, while the eastward displacement
        (d_east) represents a positive change in longitude.

        :param original_location: The original location as a LocationGlobal object (latitude, longitude, altitude)
        :type original_location: LocationGlobal or tuple or list
        :param d_north: The northward displacement in meters (positive for increasing latitude)
        :type d_north: float
        :param d_east: The eastward displacement in meters (positive for increasing longitude)
        :type d_east: float
        :return: A new LocationGlobal object or tuple or list representing the updated location
        :rtype: same as original_location
        """

        if isinstance(original_location, LocationGlobal):
            original_lat = original_location.lat
            original_lon = original_location.lon
            original_alt = original_location.alt
        else:
            original_lat = original_location[0]
            original_lon = original_location[1]
            original_alt = None

        earth_radius = 6378137.0
        # Coordinate offsets in radians
        d_lat = d_north / earth_radius
        d_lon = d_east / (earth_radius * math.cos(math.pi * original_lat / 180))
        # New position in decimal degrees
        new_lat = original_lat + (d_lat * 180 / math.pi)
        new_lon = original_lon + (d_lon * 180 / math.pi)

        if isinstance(original_location, LocationGlobal):
            return LocationGlobal(new_lat, new_lon, original_alt)
        else:
            return new_lat, new_lon

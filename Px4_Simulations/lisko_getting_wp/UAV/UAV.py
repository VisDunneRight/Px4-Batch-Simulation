
import json
from pyulog.core import ULog
import constants
# from . import constants
from collections import OrderedDict
import os
import numpy as np
from typing import List
import matplotlib.pyplot as plt
import pandas as pd


class UAV:

    def __init__(self, ulog_id: str, ulog_dir: str) -> None:
        self.ulog_dir = ulog_dir
        self.ulog_id = ulog_id
        self.ulog_data = self.extract_ulg_data()
        self.data_dict = self.get_data_dict()
        self.data_keys = list(self.data_dict.keys()
                              ) if self.data_dict else None
        self.data_set = None
        self.flights_xyz = None
        self.waypoints = None
        self.local_coords = None
        self.wp_vel = None
        self.yaw = None

    def extract_ulg_data(self) -> List[ULog]:
        try:
            abs_path = os.path.abspath(self.ulog_dir)
            print("ABS", abs_path)
            file_name = f"{self.ulog_id}.ulg"
            return ULog(os.path.join(abs_path, file_name)).data_list
        except:
            raise f"NO .ulg for {self.ulog_id }"

    def get_data_dict(self):

        try:
            data_dict = OrderedDict()
            for d in self.ulog_data:
                data_items_list = [f.field_name for f in d.field_data]
                data_items_list.remove('timestamp')
                data_items_list.insert(0, 'timestamp')
                data_items = [(item, str(d.data[item].dtype), str(
                    len(d.data[item]))) for item in data_items_list]

                i = 0
                name = d.name

                while True:
                    if i > 0:
                        name = d.name + '_' + str(i)
                    if name in data_dict:
                        i += 1
                    else:
                        break
                data_dict.setdefault(name, data_items[1:])
            return data_dict
        except:
            return None

    def set_dataset(self, dataset_name: str) -> None:

        try:
            index = self.data_keys.index(dataset_name)
            self.data_set = self.ulog_data[index]
        except Exception as ex:
            print(ex)
            print("No ulog data")

    def set_dataset_keys(self):
        try:
            data = self.data_set.data
            return data.keys()
        except:
            return None

    def get_pt_velocities(self):
        self.set_dataset("vehicle_local_position_setpoint")
        data = self.data_set.data
        wp_time_timestamps = self.get_time_stamp("position_setpoint_triplet")

        vel_x = data["vx"]
        vel_y = data["vy"]
        vel_z = data["vz"]

        return np.sqrt(np.square(vel_x) + np.square(vel_y) + np.square(vel_z))

    def set_wp_velocities(self):
        wp_time_timestamps = self.get_time_stamp("position_setpoint_triplet")
        modes = np.array(self.modes_nearest_indices(
            "position_setpoint_triplet"))
        bool_modes = modes == constants.UAV_STATUS["Mission"]
        # Used to get set velocities
        set_pt_timestamps = self.get_time_stamp(
            "vehicle_local_position_setpoint")

        set_pt_velocity = np.array(self.get_pt_velocities())
        set_pt_velocity = set_pt_velocity[~np.isnan(set_pt_velocity)]

        # Gets the index position for set waypoints
        indices = self.find_nearest_indices(
            set_pt_timestamps,
            wp_time_timestamps[bool_modes]
        )
        self.wp_vel = self.grab_data_pts(indices, set_pt_velocity)

    def grab_data_pts(self, indices: List[int], data: List) -> List:
        pts_of_interest = []
        prev_i = 0

        for i, curr_i in enumerate(indices):

            if curr_i >= prev_i:
                avg_val = np.median(data[prev_i:])

            else:
                # Get last median value range.
                # print(prev_i, curr_i)
                if len(data[prev_i:curr_i]) <= 0:
                    continue
                avg_val = max(data[prev_i:curr_i])

            prev_i = curr_i
            pts_of_interest.append(avg_val)
        return pts_of_interest

    def find_nearest_indices(self, arr_time, wp_time) -> List[float]:
        indices = []

        for target_val in wp_time:
            difference_arr = np.absolute(arr_time - target_val)
            index = difference_arr.argmin()

            indices.append(index)

        return indices

    def modes_nearest_indices(self, data_set: str) -> list:
        '''
        returns indices  of a dataset that are in the range of the UAV flight mode status.
        The indices found based on their nearest timestamp to the given flight mode
        '''

        # Get the data for the flight mode
        modes = self.get_flight_modes()
        # bool_modes = modes == mode
        # Time stamps
        mode_ts = self.get_time_stamp("vehicle_status")
        data_ts = self.get_time_stamp(data_set)

        # Indices of the selected mode
        indices = np.array(self.find_nearest_indices(
            mode_ts, data_ts))

        return [modes[ind] for ind in indices]

    def get_flight_modes(self) -> List[int]:
        self.set_dataset("vehicle_status")
        return self.data_set.data["nav_state"]

    def set_wp_altitudes(self) -> None:
        try:
            self.set_dataset("position_setpoint_triplet")
            data = self.data_set.data
            modes = np.array(self.modes_nearest_indices(
                "position_setpoint_triplet"))

            bool_modes = modes == constants.UAV_STATUS["Mission"]
            self.wp_alt = data["current.alt"][bool_modes]
        except Exception as ex:
            print(ex)

    def set_wp_coords(self) -> None:
        try:
            self.set_dataset("position_setpoint_triplet")
            data = self.data_set.data

            # Get waypoints in mission mode
            modes = np.array(self.modes_nearest_indices(
                "position_setpoint_triplet"))

            bool_modes = modes == constants.UAV_STATUS["Mission"]

            lat, lon = data["current.lat"][bool_modes], data["current.lon"][bool_modes]
            lat = np.deg2rad(lat)
            lon = np.deg2rad(lon)
            anchor_lat, anchor_lon = lat[0], lon[0]

            self.set_dataset("vehicle_local_position")
            data = self.data_set.data

            indices = np.nonzero(data["ref_timestamp"])

            # TODO Not sure why this happens
            if len(indices[0]) > 0:
                anchor_lat = np.deg2rad(data['ref_lat'][indices[0][0]])
                anchor_lon = np.deg2rad(data['ref_lon'][indices[0][0]])

            lat, lon = map_projection(lat, lon, anchor_lat, anchor_lon)
            self.waypoints = {"lat": lat, "lon": lon}

        except Exception as ex:
            print(ex)

    def set_local_coords(self) -> None:
        """_summary_
        set the local coordinates in the x,y, z and plane.
        """
        self.set_dataset("vehicle_local_position")
        data = self.data_set.data
        local_coords = dict()

        for axis in ["x", "y", "z"]:
            local_coords[axis] = [float(datum) for datum in data[axis]]

        self.local_coords = local_coords

    def set_yaw(self):
        # I found the data via a graph
        try:
            wp_timestamp = self.get_time_stamp("position_setpoint_triplet")
            modes = np.array(self.modes_nearest_indices(
                "position_setpoint_triplet"))
            bool_modes = modes == constants.UAV_STATUS["Mission"]

            arr_timestamp = self.get_time_stamp("vehicle_attitude_setpoint")
            self.set_dataset("vehicle_attitude_setpoint")
            indices = self.find_nearest_indices(
                arr_time=arr_timestamp,
                wp_time=wp_timestamp[bool_modes]
            )

            data = self.grab_data_pts(
                indices=indices, data=self.data_set.data["yaw_body"])

            self.yaw = np.rad2deg(data)

        except Exception as ex:
            print(ex)

    def get_time_stamp(self, data_name: str):
        self.set_dataset(data_name)
        return self.data_set.data["timestamp"]

    def create_data_dict(self, include_local: bool = False) -> None:

        tmp_dict = {
            "wp":
                {"wp_x": [float(val) for val in self.waypoints["lon"]],
                 "wp_y": [float(val) for val in self.waypoints["lat"]],
                 "wp_altitude_m": [float(val) for val in self.wp_alt],
                 "wp_speed_ms": [float(val) for val in self.wp_vel],
                 "wp_yaw_deg": [float(val) for val in self.yaw]}
        }

        if include_local and self.local_coords:
            tmp_dict["local"] = {
                "local_coords": self.local_coords,
                "uav_status": self.modes_nearest_indices("vehicle_local_position")
            }

        return tmp_dict

    def generate_json_file(self, directory: str, include_local: bool = False) -> None:
        if not os.path.isdir(directory):
            os.mkdir(directory)

        save_path = os.path.join(directory, f"{self.ulog_id}.json")
        tmp_dict = self.create_data_dict(include_local=include_local)
        with open(save_path, "w") as outfile:
            json.dump(tmp_dict, outfile)

    def set_all_data(self):
        self.set_wp_coords()
        self.set_wp_altitudes()
        self.set_wp_velocities()
        self.set_yaw()
        self.set_local_coords()


################# Helper Methods #####################################
def map_projection(lat, lon, anchor_lat, anchor_lon):
    """ convert lat, lon in [rad] to x, y in [m] with an anchor position """
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    cos_d_lon = np.cos(lon - anchor_lon)
    sin_anchor_lat = np.sin(anchor_lat)
    cos_anchor_lat = np.cos(anchor_lat)

    arg = sin_anchor_lat * sin_lat + cos_anchor_lat * cos_lat * cos_d_lon
    arg[arg > 1] = 1
    arg[arg < -1] = -1

    np.set_printoptions(threshold=os.sys.maxsize)
    c = np.arccos(arg)
    k = np.copy(lat)
    for i in range(len(lat)):
        if np.abs(c[i]) < np.finfo(float).eps:
            k[i] = 1
        else:
            k[i] = c[i] / np.sin(c[i])

    x = k * (cos_anchor_lat * sin_lat - sin_anchor_lat * cos_lat * cos_d_lon) * \
        constants.RADIUS_OF_EARTH
    y = k * cos_lat * np.sin(lon - anchor_lon) * constants.RADIUS_OF_EARTH

    return x, y


def main():
    mUAV = UAV("1ed4b56a-e975-4f14-99e2-d19d69df3120", "./dataDownloaded/")
    mUAV.set_wp_coords()
    mUAV.set_wp_altitudes()
    mUAV.set_wp_velocities()
    mUAV.set_yaw()
    mUAV.set_local_coords()

    # out_dict = mUAV.create_data_dict(True)

    # print(out_dict['local'])

    # modes = mUAV.modes_nearest_indices("vehicle_local_position")

    # i = 0
    # tmp_dict = []
    # local = mUAV.local_coords
    # for mode in modes:
    #     tmp_dict.append({"mode": mode, "x": local["x"][i], "y": local["y"][i]})
    #     i += 1

    # df = pd.DataFrame.from_dict(tmp_dict)

    # grps = np.unique(modes)
    # for mode in grps:
    #     tmp = df[df["mode"] == mode]
    #     plt.plot(tmp["y"], tmp["x"])

    # plt.scatter(mUAV.waypoints["lon"], mUAV.waypoints["lat"])

    # plt.show()

    # mUAV.generate_json_file("./saved_waypoints")


if __name__ == '__main__':
    main()

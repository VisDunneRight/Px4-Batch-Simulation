# Px4-Batch-Simulation
This python script runs batch simulations on the PX4 platform.

The python script uses MavSDK as middleware communication to execute mission plans.
The script is based on Px4 MavSDK testing found [here](https://github.com/PX4/PX4-Autopilot/blob/080ba4458a1ad2015a92110cd0b9815982ead04c/test/mavsdk_tests/mavsdk_test_runner.py#L136).
The current version works with Gazebo and other simulators planned for the future.

## Requirements
To use this simulation code, you will need to install several libraries. 

### PX4
There are several ways to install PX4, and guides on how to do it can be found on their [site](https://docs.px4.io/master/en/simulation/gazebo.html).

### MAVSDK

To install mavsdk-python, run:

```
pip3 install mavsdk
```


## Running the simulation
To run the simulation and series of examples:

```
python3 Simulation.py config.json
```

This execution assumes PX4 is located in px4Developer/Firmware/ from the local directory.
To provide the script a different build direction, run:

```
python3 Simulation.py --build-dir where/PX4/Fireware/Located config.json
```

The config.json tells the script which missions to run and their order.

```
{
  "mavlink_connection": "",
  "test_directory":"missions/",
  "tests":[
    {
      "name":"Mission 1: Takeoff and land",
      "excutable": "sdk_takeoff_and_land.py"
    },
    {
      "name":"Mission 2: Square flight plan",
      "excutable": "sdk_square_mission.py"
    },
    ...
```
mavlink_connection will hold information connection information:
default:udp://:14540 
Test_directory is where the missions are stored.
"tests" is an array that holds the mission's name and executable location.

## File Structure

### Simulation.py
The main file that runs PX4 and the individual test cases. Currently only works on Gazebo but will be extended in the future to support JMavSim and AirSim.

### MissionHelper.py and MissionHelperSDK.py
Helper class for creating missions.

### missions/sdk_* or missions/fly_*
Example missions are written for MavSDK and DroneKit. The config.json points to these executables to run test cases.
These files can be run independently but require PX4 to be run separately.




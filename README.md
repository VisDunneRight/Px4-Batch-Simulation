# Px4-Batch-Simulation
This python script to run batch simulations on the PX4 platform.

The python script use mavSDK as middleware communication to excute mission plans.
The script is based on Px4 mavsdk testing found [here](https://github.com/PX4/PX4-Autopilot/blob/080ba4458a1ad2015a92110cd0b9815982ead04c/test/mavsdk_tests/mavsdk_test_runner.py#L136).
Current version works with gazebo with other simulator planned in the future.

## Requirements
To use this simulation code, you will need to install several libraries. 

### PX4
There are several ways to install PX4 and guides to how to do it can be found on their [site](https://docs.px4.io/master/en/simulation/gazebo.html).

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

This excution assumes that PX4 is located in px4Developer/Firmware/ from the local directory.
To provide the script a different build direction run:

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
```
mavlink_connection will hold information connection information:
default:udp://:14540 
Test_directory is where the mission are stored.
Tests is an array that holds the name of the mission and excutable location.

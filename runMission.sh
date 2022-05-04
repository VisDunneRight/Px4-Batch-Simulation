#!/bin/bash
screen -S Px4Gazebo -d -m -c 'HEADLESS=1 make px4_sitl gazebo'
sleep 60
python3 ../../DroneKit-mission.py
screen -S Px4Gazebo -X quit
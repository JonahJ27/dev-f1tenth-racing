# Development for F1tenth Racing

## Overview
This is a github repository for the development of f1tenth racing algorithms.
The ROS2 nodes developed are meanth to run in the 
[f1tenth gym simulation](https://github.com/f1tenth/f1tenth_gym_ros).
The initial plan is to develop greedy gap following algorithms as well as
map based algorithms for navigation, and then to use reinforcement learning to 
improve those algorithms. Materials used from https://f1tenth.org/ and numpy reinforcement 
learning adapted from https://github.com/philtabor/Youtube-Code-Repository/blob/master/ReinforcementLearning/Fundamentals/mountaincar.py


## Safety
In order to ensure safety of a self driving car, I have implemented autonomous 
emergincy breaking by:
* Obtaining necessary data using a lidar scan and the car's odometry data.
* Projecting each lidar laser in the direction of the car.
* Calculating the time to collision for each lidar laser.
* Publishing a brake message is the time to collision is below a threshold.

This node is implemented in both python and c++. 
Run each node respectively using:
```
ros2 run safety aeb
```
```
ros2 run cpp_safety safety
```

## Reactive Gap Follow
In situations where a car has no map and needs to race, I have implemented a
reactive gap following algorithm by:
* Obtaining necessary data using a lidar scan.
* Preprocessing it by setting a max distance to crop gaps to.
* Further processing the data by creating a bubble around the closest point
to avoid.
* Finding the gap of largest size, and heading towards the center of it.
* Note there are other implementations that head towards a different part of the 
gap based upon various factors, but I've found this to be most effective for 
speed and reliability.

This node is implemented in both python and c++. 
Run each node respectively using:
```
ros2 run gap_follow reactive_gap_follow
```
```
ros2 run cpp_gap_follow rgf
```

## Pure Pursuit
Currently in progress. Using two different algorithms for selecting points to follow
and a few algorithms for generating points to varying amounts of success by:
* Generating a set of waypoints using the path folder.
* Algorithm 1: Generate a circle of a certain size around the car and have the car path to the point on that circle directly between the closest point inside the circle and the closest point outside of the circle.
* Algorithm 2: path to the closest point outside of the circle.

This node is implemented in python 
Run each algorithm respectively using:
```
ros2 run pursuit pure_pursuit [waypoints_filename].csv
```
```
ros2 run pursuit mod_pursuit [waypoints_filename].csv
```
Waypoint files live in the path/waypoint-files folder. So far path-clean-speilburg.csv has been the most effective waypoint file, but there are better options to be made.

## Q Learning Gap Follow
Typical gap following algorithms tend to have some issues taking the most direct path on straight tracks, I am trying to alieviate this by:
* Implementing a modified reinforcement learning algorithm using numpy.
* Feeding thaat algorithm the same input of the start and end index of the max gap.
* Having it learn from 40% randomly generated moves and 60% reactive gap follow.
* Eventually having it learn from those random moves to be faster than normal reactive gap follow.

The node is implemented in python and can be run using:
```
ros2 run q_gap_follow gap_reinforcement
```
The best result so far had the car learn to go from driving around the track in 15,000
callbacks to 12,000 consistently, but I'm having some issues with the car jailbreaking the walls in the simulation and driving impossibly fast laps.
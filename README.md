# Development for F1tenth Racing

## Overview
This is a github repository for the development of f1tenth racing algorithms.
The ROS2 nodes developed are meanth to run in the 
[f1tenth gym simulation](https://github.com/f1tenth/f1tenth_gym_ros).
The initial plan is to develop greedy gap following algorithms as well as
map based algorithms for navigation, and then to use immitation learning 
followed by reinforcement learning to improve those algorithms.


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
In situations where a car has no map and need to race, I have implemented a
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
and a few algorithms for generating points to varying amounts of success.
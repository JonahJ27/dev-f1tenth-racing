#!/usr/bin/env python
import rclpy
import numpy as np
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node

class waypoints(Node):
    def __init__(self, file):
        super().__init__('reactive_follow_gap')
        self.f = file
        self.create_subscription(Odometry, 
                                 '/ego_racecar/odom', 
                                 self.save_waypoint,
                                 11)



    # uses tf_transform quaternions to generate waypoins based on car odometry.
    # note due to packages in ROS2 quaternions are in [w, x, y, z] format.
    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.w,
                               data.pose.pose.orientation.x, 
                               data.pose.pose.orientation.y, 
                               data.pose.pose.orientation.z])

        euler = euler_from_quaternion(quaternion)

        speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                 data.twist.twist.linear.y, 
                                 data.twist.twist.linear.z]),2)
        if data.twist.twist.linear.x>0.:
            print(quaternion)
            print(euler)

        self.f.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                     data.pose.pose.position.y,
                                     euler[2],
                                     speed))

# shuts down the program and closes the file
def shutdown(file):
    file.close()
    print('Goodbye')
    

def main():
    print('Saving waypoints...')
    home = expanduser('~')
    file = open(strftime(home+'/dev_f1tenth_ws/src/path/waypoint-files/path-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

    rclpy.init(args=None)
    waypoint = waypoints(file)
    rclpy.spin(waypoint)

    shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from os.path import expanduser
import sys

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from tf_transformations import euler_from_quaternion

class PurePursuit(Node):
    def __init__(self, lookahead_distance, filename):
        super().__init__('mod_pursuit_node')
        self.ld = lookahead_distance
        self.curr_ld = lookahead_distance
        home = expanduser('~')
        file = open(home + "/dev_f1tenth_ws/src/path/waypoint-files/" + filename,'r')
        self.waypoints = [] 
        for line in file.readlines():
            vals = line.split(",")
            self.waypoints.append([float(vals[0]), float(vals[1])])

        self.waypoint_idx = 0
        # create ROS subscribers and publishers
        self.create_subscription(Odometry, 
                                 '/ego_racecar/odom', 
                                 self.pose_callback,
                                 11)

        node = rclpy.create_node('ackermann_drive')
        self.drive_pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)

    def pose_callback(self, pose_msg):
        [x, y] = self.find_target_point(pose_msg.pose.pose.position.x,
                                   pose_msg.pose.pose.position.y)
        
        [x, y] = self.transform_to_vehicle_frame(pose_msg, x, y)
        
        # calculate curvature/steering angle
        theta = 2 * (y - pose_msg.pose.pose.position.y) / (self.ld ** 2)
        theta = min(max(-np.pi / 2, theta), np.pi / 2)
        # print((x, y, theta, pose_msg.pose.pose.position.x,
        #                         pose_msg.pose.pose.position.y))

        # publish drive message
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = theta
        msg.drive.speed = min(0.3 / theta, 5.0)
        self.drive_pub.publish(msg)



    # transform goal point to vehicle frame of reference
    def transform_to_vehicle_frame(self, pose_msg, x, y):
        quaternion = np.array([pose_msg.pose.pose.orientation.w,
                               pose_msg.pose.pose.orientation.x, 
                               pose_msg.pose.pose.orientation.y, 
                               pose_msg.pose.pose.orientation.z])

        euler = euler_from_quaternion(quaternion)
        angle = -1 * (euler[0] * -1 + np.pi)

        # print((pose_msg.pose.pose.position.x,
        #                            pose_msg.pose.pose.position.y,
        #                            x, y, angle), rotate_around_point(pose_msg.pose.pose.position.x,
        #                            pose_msg.pose.pose.position.y,
        #                            x, y, angle))

        return rotate_around_point(pose_msg.pose.pose.position.x,
                                   pose_msg.pose.pose.position.y,
                                   x, y, angle)


    # find the current waypoint to track 
    # methodology: finds the closest point in and outside of the lookup distance
    # then picks the closer one to the circle
    def find_target_point(self, x_pos, y_pos):
        max_dist = self.dist_posn_to_point(self.waypoints[self.waypoint_idx], x_pos, y_pos)
        if max_dist > self.ld:
            return self.waypoints[self.waypoint_idx]

        while self.ld > max_dist:
            self.waypoint_idx = (self.waypoint_idx + 1) % len(self.waypoints)
            max_dist = self.dist_posn_to_point(self.waypoints[self.waypoint_idx], x_pos, y_pos)


        return self.calc_waypoint(self.waypoints[self.waypoint_idx], 
            self.waypoints[(self.waypoint_idx + 1) % len(self.waypoints)], 
            x_pos, y_pos)

    # calculates the waypoint closest to the lookup distance
    # mutates seld.curr_ld accordingly
    def calc_waypoint(self, in_pos, out_pos, x_pos, y_pos):
        in_dist = self.dist_posn_to_point(in_pos, x_pos, y_pos)
        out_dist = self.dist_posn_to_point(out_pos, x_pos, y_pos)

        if in_dist >= out_dist:
            self.curr_ld = in_dist
            return in_pos
        else:
            self.curr_ld = out_dist
            return out_pos


    # dist from position to current point
    def dist_posn_to_point(self, posn, x_center, y_center):
        return math.sqrt((posn[0] - x_center) ** 2 
            + (posn[1] - y_center) ** 2)

# finds the roots of a quaratic equation with real roots
def quadratic_find_roots(a, b, c):
    dis_form = b * b - 4 * a * c  
    sqrt_val = math.sqrt(abs(dis_form)) 
    if dis_form < 0:
        raise Exception("complex roots")

    return [(-b + sqrt_val) / (2 * a), (-b - sqrt_val) / (2 * a)]

# rotates a point arount a pivot point by a given theta
def rotate_around_point(pivot_x, pivot_y, rot_x, rot_y, theta):
    sin = np.sin(theta)
    cos = np.cos(theta)

    rot_x -= pivot_x
    rot_y -= pivot_y

    x = rot_x * cos - rot_y * sin
    y = rot_x * sin + rot_y * cos

    x += pivot_x
    y += pivot_y

    return [x, y]


def main(args=sys.argv):
    rclpy.init(args=None)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit(2, args[1])
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

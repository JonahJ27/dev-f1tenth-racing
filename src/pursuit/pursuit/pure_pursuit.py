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
        super().__init__('pure_pursuit_node')
        self.ld = lookahead_distance
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

        print((pose_msg.pose.pose.position.x,
                                   pose_msg.pose.pose.position.y,
                                   x, y, angle), rotate_around_point(pose_msg.pose.pose.position.x,
                                   pose_msg.pose.pose.position.y,
                                   x, y, angle))

        return rotate_around_point(pose_msg.pose.pose.position.x,
                                   pose_msg.pose.pose.position.y,
                                   x, y, angle)


    # find the current waypoint to track 
    # methodology: we assume the previous fathest point within the lookahead 
    # cicle is always further than any other points previous to it in the list
    # we then look ahead until we find a point out of the circle. Then
    # take the point in the lookahead circle intersecting a line between closest in
    # and out of the circle
    def find_target_point(self, x_pos, y_pos):
        furthest_in_circle = self.waypoints[self.waypoint_idx]
        pos = self.waypoint_idx
        max_dist = self.dist_posn_to_point(self.waypoints[pos], x_pos, y_pos)
        curr_dist = self.dist_posn_to_point(self.waypoints[pos], x_pos, y_pos)

        while self.ld > curr_dist:
            if max_dist < curr_dist:
                self.waypoint_idx = pos
                max_dist = curr_dist
            pos = (pos + 1) % len(self.waypoints)
            curr_dist = self.dist_posn_to_point(self.waypoints[pos], x_pos, y_pos)

        # car has moved such that the current waypoint idx is further than ld
        if self.waypoint_idx == pos:
            print(str((self.waypoints[self.waypoint_idx], self.waypoints[pos])))

        return self.calc_waypoint(self.waypoints[self.waypoint_idx], self.waypoints[pos], 
            x_pos, y_pos)

    # calculates the waypoint on the circle of radius lookup distance
    def calc_waypoint(self, in_pos, out_pos, x_pos, y_pos):
        if abs(out_pos[0] - in_pos[0]) > 0.00001:
            slope = (out_pos[1] - in_pos[1]) / (out_pos[0] - in_pos[0])
            y_intercept = out_pos[1] - out_pos[0] * slope
            y_offset = y_intercept - y_pos

            [sol1, sol2] = quadratic_find_roots(slope * slope + 1,
                2 * ((slope * y_offset) - x_pos), x_pos ** 2 + y_offset ** 2 - self.ld ** 2)

            if sol1 > min(in_pos[0], out_pos[0]) and sol1 < max(in_pos[0], out_pos[0]):
                return [sol1, slope * sol1 + y_intercept]
            elif sol2 > min(in_pos[0], out_pos[0]) and sol2 < max(in_pos[0], out_pos[0]):
                return [sol2, slope * sol2 + y_intercept]
            else:
                raise Exception("no overlapping point:" + str((sol1, sol2, 
                in_pos[0],
                out_pos[0])))
        elif abs(out_pos[1] - in_pos[1]) > 0.00001:
            print("y used")
            slope = (out_pos[0] - in_pos[0]) / (out_pos[1] - in_pos[1])
            x_intercept = out_pos[0] - out_pos[1] * slope
            x_offset = x_intercept - x_pos

            [sol1, sol2] = quadratic_find_roots(slope * slope + 1,
                2 * ((slope * x_offset) - y_pos), y_pos ** 2 + x_offset ** 2 - self.ld ** 2)

            if sol1 > min(in_pos[1], out_pos[1]) and sol1 < max(in_pos[1], out_pos[1]):
                return [slope * sol1 + x_intercept, sol1]
            elif sol2 > min(in_pos[1], out_pos[1]) and sol2 < max(in_pos[1], out_pos[1]):
                return [slope * sol2 + x_intercept, sol2]
            else:
                raise Exception("no overlapping point:" + str((sol1, sol2, 
                in_pos[0],
                out_pos[0])))
        else:
            print("highly overlapping waypoints:" + str((in_pos, out_pos)))
            return in_pos

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

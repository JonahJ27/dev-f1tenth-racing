#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rclpy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from rclpy.node import Node

# nunmbers to tweak: 
# the 3 as the max number of meters to set to
# the 30 in the number of ranges to change in the lidar bubble
# the find max gap threshhold

class reactive_follow_gap(Node):
    def __init__(self, max_dist):
        super().__init__('reactive_follow_gap')
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            10)

        node = rclpy.create_node('ackermann_drive')
        self.drive_pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.max_dist = max_dist

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # sets max ranges to global max
        proc_ranges = list(map(lambda r: min(r, self.max_dist), ranges))

        
        # sets a mean of the 5 items on either side and this value to this one
        final_ranges = []
        for i in range(len(proc_ranges)):
            window = []
            for j in range(-5, 6):
                if i + j >= 0 and i + j < len(proc_ranges):
                    window.append(proc_ranges[i + j])
            final_ranges.append(sum(window) / len(window))

        return final_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        start_gaps = []
        end_gaps = []
        threshold = max(free_space_ranges) - 0.5
        in_gap = False

        for i in range(len(free_space_ranges)):
            if in_gap and free_space_ranges[i] < threshold:
                in_gap = False
                end_gaps.append(i)
            if not in_gap and free_space_ranges[i] >= threshold:
                in_gap = True
                start_gaps.append(i)

        if len(start_gaps) > len(end_gaps):
            end_gaps.append(len(free_space_ranges) - 1)

        max_idx = np.argmax(list(map(lambda start, end: end - start, start_gaps, end_gaps)))

        return [start_gaps[max_idx], end_gaps[max_idx]]
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
        """

        # current greedy decision, drive towars the middle of the gap
        return  (end_i - start_i) / 2 + start_i

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closest_dist = min(proc_ranges)
        #Eliminate all points inside 'bubble' (set them to zero) 
        closest_idx = proc_ranges.index(closest_dist)
        for i in range(-1 * int(np.ceil(30 / closest_dist)) + closest_idx,
                        1 + int(np.ceil(30 / closest_dist)) + closest_idx):
            if i >= 0 and i < len(proc_ranges):
                proc_ranges[i] = 0
        #Find max length gap 
        [start_i, end_i] = self.find_max_gap(proc_ranges)
        #Find the best point in the gap 
        best_point = self.find_best_point(start_i, end_i, proc_ranges)
        print(best_point)
        #Publish Drive message
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = data.angle_min + data.angle_increment * best_point
        print(data.angle_min + data.angle_increment * best_point)
        msg.drive.speed = np.pi - abs(msg.drive.steering_angle)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rfgs = reactive_follow_gap(3)
    rclpy.spin(rfgs)

if __name__ == '__main__':
    main()
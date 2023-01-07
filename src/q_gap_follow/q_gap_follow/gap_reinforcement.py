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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# nunmbers to tweak: 
# the 3 as the max number of meters to set to
# the 30 in the number of ranges to change in the lidar bubble
# the find max gap threshhold

class reactive_follow_gap(Node):
    def __init__(self, max_dist):
        super().__init__('reactive_follow_gap')
        # q learning parameters 

        # our gap endpoints are between 408 and 777 in our normal rgf so we give a little
        # buffer and limit the endpoints to 400 to 800
        # we do the same for states
        [self.Q, self.action_space] = initialize_q(400, 800)
        
        self.game = 0
        self.score = 0
        self.n_games = 1000
        self.total_rewards = np.zeros(self.n_games)
        self.alpha = 0.1
        self.gamma = 0.99
        self.eps = 1.0
        
        # transition parameters
        self.prev_state = (408, 777)
        self.can_finish = False
        # ros parameters
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom', 
            self.odom_callback,
            10)

        node = rclpy.create_node('ackermann_drive')
        self.drive_pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.reset_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
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

        return (max(400, min(start_gaps[max_idx], 799)), \
            max(400, min(end_gaps[max_idx], 799)))
    
    # Start_i & end_i are start and end indicies of max-gap range, respectively
    def find_best_point(self, start_i, end_i):
        # current greedy decision, drive towars the middle of the gap
        return  (end_i - start_i) / 2 + start_i

    # resets after a lap
    def reset(self):
        print('episode ', self.game, 'score ', self.score, 'epsilon %.3f' % self.eps)
        self.prev_state = (408, 777)
        self.total_rewards[self.game] = self.score
        self.eps = self.eps - 1.5/self.n_games if self.eps > 0.01 else 0.01
        self.can_finish = False
        self.game += 1
        self.score = 0
        reset = PoseWithCovarianceStamped()
        reset.pose.pose.position.x = 0.0
        reset.pose.pose.position.y = 0.0
        reset.pose.pose.position.z = 0.0
        reset.pose.pose.orientation.x = 0.0
        reset.pose.pose.orientation.y = 0.0
        reset.pose.pose.orientation.z = 0.0
        self.reset_pub.publish(reset)

        if self.game >= self.n_games:
            raise Exception(str(self.total_rewards))

    # just to check if we 
    def odom_callback(self, data):
        pos = np.array((data.pose.pose.position.x, data.pose.pose.position.y))
        if math.sqrt(pos[0] ** 2 + pos[1] ** 2) > 30 and not self.can_finish:
            print("Hit Checkpoint")
            self.can_finish = True

        if self.can_finish and math.sqrt(pos[0] ** 2 + pos[1] ** 2) < 4 \
                and pos[0] >=0 and pos[1] >= 0:
            self.reset()
        if self.score < -20000:
            self.score -= 30000
            self.can_finish = False
            self.reset()

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
        #Find max length gap and cache it
        state = self.find_max_gap(proc_ranges)

        # if greate than epsilon, perform max action, else 40% chance to use greedy, 60% random
        action = self.find_best_point(state[0], state[1]) \
                if np.random.random() < 0.6 \
                else np.random.choice(self.action_space)
        action = max_action(self.Q, state, self.action_space) \
                    if np.random.random() > self.eps else int(action)
        
        reward = -1
        self.score -= 1
        self.Q[state, action] = self.Q[state, action] + \
                    self.alpha*(reward + self.gamma * self.Q[self.prev_state, 
                        max_action(self.Q,self.prev_state, self.action_space)] \
                        - self.Q[state, action])
        #Publish Drive message
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = data.angle_min + data.angle_increment * action
        msg.drive.speed = np.pi - abs(msg.drive.steering_angle)
        self.drive_pub.publish(msg)


def max_action(Q, state, actions):
    values = np.array((Q[state,a] for a in actions))
    action = np.argmax(values) + 400

    return action


def initialize_q(min_actions, max_actions):
    action_space = []
    for action in range(min_actions, max_actions):
        action_space.append(action)

    states = []
    for start in range(min_actions, max_actions):
        for end in range(start, max_actions):
            states.append((start, end))

    Q = {}
    for state in states:
        if state[1] == 799:
            print(state[0])
        for action in action_space:
            Q[state, action] = 0

    return Q, action_space

def main(args=None):
    


    rclpy.init(args=args)
    rfgs = reactive_follow_gap(3)
    rclpy.spin(rfgs)

if __name__ == '__main__':
    main()
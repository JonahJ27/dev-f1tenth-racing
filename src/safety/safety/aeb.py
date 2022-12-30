import numpy as np
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node

class LaserScanAEB(Node):

    def __init__(self):
        super().__init__('cv_control_publisher_subscriber')
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan', 
            self.laser_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom', 
            self.odom_callback,
            10)
        node = rclpy.create_node('teleop_twist')
        self.pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.speed = 0.0
        self.ttc_threshold = 1
        # to prevent errors for unused variables
        self.laser_sub
        self.odom_sub
        

    def laser_callback(self, msg):
        distances = msg.ranges
        min_ttc = timeToCollision(self.speed, distances[0], msg.angle_min)
        for i in range(1, len(distances)):
            cur_ttc = time_to_collision(self.speed, distances[i], msg.angle_min + msg.angle_increment * i)
            if cur_ttc <  min_ttc:
                min_ttc = cur_ttc

        if (min_ttc < self.ttc_threshold):
            drive = AckermannDriveStamped()
            drive.drive.speed = 0.0
            drive.drive.acceleration = 0.0
            drive.drive.jerk = 0.0
            self.pub.publish(drive)
            print(min_ttc)

    def odom_callback(self, msg):
        vector = msg.twist.twist.linear
        self.speed = np.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)
        print(self.speed) 

# ||  u * v      ||
# ||--------- * v|| = ||u|| cos(theta)
# || ||v^2||     ||
# ttc = || v || / max(proj, 0)
def time_to_collision(u, v, theta):
    proj = u * np.cos(theta)
    if proj <= 0:
        return 1000000.0
    return v / proj


def main(args=None):
    rclpy.init(args=args)

    laser_scan_aeb = LaserScanAEB()

    rclpy.spin(laser_scan_aeb)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser_scan_aeb.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

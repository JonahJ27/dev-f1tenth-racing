#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <iostream>
#include <cmath>
#include <memory>
using namespace std;

class LaserScanAEB : public rclcpp::Node {
// The class that handles emergency braking

public:
    LaserScanAEB() : Node("safety_node")
    {
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
      
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10, std::bind(&LaserScanAEB::drive_callback, this, std::placeholders::_1));

      laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LaserScanAEB::scan_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "AEB Initialized");
    }

private:
    double speed = 0.0;
    const double ttcThreshold = 1.0;


    // updates the speed
    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        speed = sqrt(pow(msg->twist.twist.linear.x, 2) +  pow(msg->twist.twist.linear.y, 2)
          + pow(msg->twist.twist.linear.z, 2));
    }

    // calculates the time to collision from the scan
    // sets the car to brake if ttc is less than the threshold
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        
        auto distances = scan_msg->ranges;
        double minTTC = time_to_collision(speed, distances[0], scan_msg->angle_min);
        int lasers = int((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment) + 1;
        for (int i = 1; i < lasers; i++) {
          double curr_ttc = time_to_collision(speed, distances[i], 
            scan_msg->angle_min + scan_msg->angle_increment * i);
          if (curr_ttc < minTTC) {
            minTTC = curr_ttc;
          }
        }

        if (minTTC < ttcThreshold) {
          auto drive = ackermann_msgs::msg::AckermannDriveStamped();
          drive.drive.speed = 0.0;
          drive.drive.acceleration = 0.0;
          drive.drive.jerk = 0.0;
          publisher_->publish(drive);
          RCLCPP_INFO(this->get_logger(), to_string(minTTC));
        }
    }

    // calculates the time to collision for the car
    double time_to_collision(double u, double v, double theta) {
        double proj = u * cos(theta);
        if (proj <= 0) {
            return 1000000;
        }
        return v / proj;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};





int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanAEB>());
    rclcpp::shutdown();
    return 0;
}
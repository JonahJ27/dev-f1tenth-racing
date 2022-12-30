#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <iostream>
#include <cmath>
#include <memory>
#include <utility> 
#include <algorithm>
#include <vector>
#include <cstdlib>
using namespace std;

class ReactiveGapFollow : public rclcpp::Node {
// The class that handles emergency braking

public:
    ReactiveGapFollow(double max_dist) : Node("safety_node")
    {
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
      
      laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ReactiveGapFollow::scan_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "AEB Initialized");

      max_dist_ = max_dist;

    }

private:
    double* preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg, int lasers) {
        auto ranges = msg->ranges;
        double* proc_ranges = new double[lasers];
        for (int i = 0; i < lasers; i++) {
            proc_ranges[i] = min(double(ranges[i]), 3.0);
        }

        double* final_ranges = new double[lasers];

        for (int i = 0; i < lasers; i++) {
            double sum = 0.0;
            int count = 0;
            for (int j = -5; j < 6; j++) {
                if (i + j >=0 && i + j < lasers) {
                    sum += proc_ranges[i + j];
                    count += 1;
                }
            }
            final_ranges[i] = sum / count;
        }
        return final_ranges;
    }

    // returns the start and end index of the maximum gap
    pair<int, int> find_max_gap(double* free_space_ranges, int lasers) {
        vector<int> start_gaps;
        vector<int> end_gaps;
        double threshold = *max_element(free_space_ranges, free_space_ranges + lasers) - 0.5;
        bool in_gap = false;

        for (int i = 0; i < lasers; i++) {
            if (in_gap && free_space_ranges[i] < threshold) {
                in_gap = false;
                end_gaps.push_back(i);
            }
            if (!in_gap && free_space_ranges[i] >= threshold){
                in_gap = true;
                start_gaps.push_back(i);
            }
        }

        if (start_gaps.size() > end_gaps.size()) {
            end_gaps.push_back(lasers - 1);
        }

        int max_gap_size = 0;
        int max_gap_idx = 0;

        for (int i = 0; i < int(start_gaps.size()); i++) {
            if (end_gaps[i] - start_gaps[i] > max_gap_size) {
                max_gap_size = end_gaps[i] - start_gaps[i];
                max_gap_idx = i;
            }
        }

        return make_pair(start_gaps[max_gap_idx], end_gaps[max_gap_idx]);
    }  


    // process each LiDAR scan as per the Follow Gap algorithm 
    // & publish an AckermannDriveStamped Message
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        int lasers = int((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;
        double* proc_ranges = preprocess_lidar(msg, lasers);

        // Find closest point to LiDAR
        double closest_dist = *min_element(proc_ranges, proc_ranges + lasers);
        // Eliminate all points inside 'bubble' (set them to zero) 
        auto itr = find(proc_ranges, proc_ranges + lasers, closest_dist);
        int closest_idx = distance(proc_ranges, itr);

        for (int i = -1 * int(ceil(30.0 / closest_dist)) + closest_idx; 
            i <= int(ceil(30.0 / closest_dist)) + closest_idx; i++) {
            if (i >= 0 && i < lasers) {
                proc_ranges[i] = 0;
            }
        }
            
        // Find max length gap 
        pair<int, int> gap = find_max_gap(proc_ranges, lasers);
        // Find the best point in the gap 
        int best_point = find_best_point(gap.first, gap.second);
        // Publish Drive message
        auto drive = ackermann_msgs::msg::AckermannDriveStamped();
        drive.drive.steering_angle = msg->angle_min + msg->angle_increment * best_point;
        RCLCPP_INFO(this->get_logger(), to_string(msg->angle_min + msg->angle_increment * best_point));
        drive.drive.speed = M_PI - abs(drive.drive.steering_angle);
        publisher_->publish(drive);
    }

    // current greedy decision, drive towars the middle of the gap
    // other options are possible, first third has had success
    int find_best_point(double start_i, double end_i) {
        return  (end_i - start_i) / 2 + start_i;
    }

        
        

    double max_dist_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};





int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveGapFollow>(10));
    rclcpp::shutdown();
    return 0;
}
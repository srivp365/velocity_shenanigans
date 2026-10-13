#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


#include <tuple>
#include <algorithm>
#include <cmath>

class ReactiveFollowGap : public rclcpp::Node {

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->lidar_callback(msg);
        });

        pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        RCLCPP_INFO(this->get_logger(), "GAP_FOLLOW ONLINE");
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_; 

    // custom vars
    std::vector<float> processed_lidar; // following waterloo's approach of using class vars instead of passing as params to functions 😁
    int range_len;

    void preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        processed_lidar = msg->ranges;
        range_len = msg->ranges.size();
        for (int i = 0; i < range_len; ++i) {
            if (!std::isfinite(processed_lidar[i])) {
                 processed_lidar[i] = 0.0f;
            }
            if (processed_lidar[i] > 3) {
                processed_lidar[i] = msg->range_max;
            }
        }
        


        return;
    }

    std::tuple<int,int> find_max_gap()
    {   
        
        int max_cnt = 0;
        int best_start = 0;
        int best_end = 0;

        int i = 0;
        while (i < range_len) {

            while (i < range_len && processed_lidar[i] == 0) {
                i++;
            }

            if (i >= range_len) break;
        
            int start = i;
            int counter = 0;

            while (i < range_len && processed_lidar[i] != 0) {
                counter++;
                i++;
            }
            int end = i - 1;
        
            if (counter >= max_cnt) {
                max_cnt = counter;
                best_start = start;
                best_end = end;
            }
    }

    
        return {best_start, best_end};
    }

    int find_best_point(const std::tuple<int, int>& gap_info)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        
        auto [start, end] = gap_info;

        int best_point = start;
        for (int i = start; i <= end; ++i) {
            if (processed_lidar[i] > processed_lidar[best_point]) {
                best_point = i;
            }
        }
        

        return best_point;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        preprocess_lidar(scan_msg);

        // roundabout way of getting a non-0.0f value from the array
        float min_dist = 10000;
        int min_idx = -1;

        for (long unsigned int i = 0; i < range_len; ++i) {
            if (processed_lidar[i] > 0.0f && processed_lidar[i] < min_dist) {
                min_idx = i;
                min_dist = processed_lidar[i];
            }
        }


        int closest_obs_idx = min_idx;

        // Eliminate all points inside 'bubble' (set them to zero), this algorithm was suggested by AI
        double half_angle = std::asin(std::min(1.0, 0.5 / processed_lidar[closest_obs_idx]));          
        int half_beams = (int)std::ceil(half_angle / scan_msg->angle_increment);
        int bubble_radius = half_beams;

        // this stuff is mine 
        for (int i = -bubble_radius; i <= bubble_radius; ++i) {
            int idx = closest_obs_idx + i;
            if (idx >= 0 && idx < range_len) {
                processed_lidar[idx] = 0.0f;
            }
        }


        // Find max length gap 
        std::tuple<int,int> gap = find_max_gap();
        

        // Find the best point in the gap 
        int point = find_best_point(gap);

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = scan_msg->angle_min + point * scan_msg->angle_increment;
        
         
        if (point == -1)
            drive_msg.drive.speed = 0.5;
        else
            drive_msg.drive.speed = 1.0;
        drive_msg.header.stamp = this->get_clock()->now();
        pub_drive_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
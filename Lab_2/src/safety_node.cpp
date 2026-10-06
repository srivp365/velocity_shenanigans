
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

class Safety : public rclcpp::Node {
private:
    double speed = 0.0;
    double ittc_threshold = 1.0;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr cmd_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_drive_pub_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        if (speed == 0.0) {
            return;
        }

        double min_ttc = std::numeric_limits<double>::infinity();

        const auto & ranges = scan_msg->ranges;
        const size_t num_angles = ranges.size();

        for (size_t i = 0; i < num_angles; ++i) {
            const double range = ranges[i];


            if (!std::isfinite(range) || range < 0.0) {
                continue;
            }

            const double angle = scan_msg->angle_min + i * scan_msg->angle_increment;


            const double speed_in_angle = speed * std::cos(angle);


            if (speed_in_angle <= 0.0) {
                continue;
            }


            const double ttc = range / speed_in_angle;

            min_ttc = std::min(min_ttc, ttc);
        }


        if (min_ttc < ittc_threshold) {
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = this->get_clock()->now();
            drive_msg.drive.speed = 0.0;

            cmd_drive_pub_->publish(drive_msg);

            RCLCPP_WARN(
                this->get_logger(),
                "AEB Triggered! iTTC: %.2fs",
                min_ttc
            );
        }
    }

public:
    Safety() : Node("safety_node")
    {
        cmd_drive_pub_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", 10);

        cmd_scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10,
                [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                    this->scan_callback(msg);
                });

        cmd_odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/ego_racecar/odom", 10,
                [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    this->drive_callback(msg);
                });
        RCLCPP_INFO(this->get_logger(), "AEB ONLINE");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}

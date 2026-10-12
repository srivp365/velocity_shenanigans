#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define _USE_MATH_DEFINES // Must be defined BEFORE including <cmath>
#include <cmath>


class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // define 1 a sub to get lidar scans
        sub_lidar_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->lidarscan_topic, 10, [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
            this->scan_callback(msg);
        });
        // define 1 a pub to push changes to /drive topic
        pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->drive_topic, 10);
        RCLCPP_INFO(this->get_logger(), "AEB ONLINE");
    }
    

private:
    // PID CONTROL PARAMS
    double kp = 1.0;
    double kd = 0.0;
    double ki = 0.0;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    // user defined vars
    double theta = M_PI / 4; // radian
    double L = 1; // m?


    // defined so get_range can access them
    double angle_min; // rads
    double angle_increment; // rads

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    // subscriptions defined
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;

    double get_range(const float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */


        // full disclosure, I did look up how to convert from angle to array index
        double phi = angle * M_PI / 180;

        // ik this is super bad error handling
        if (i < 0 || std::isnan(phi)) {
            phi = 0;
        }

        int i = std::round((phi - angle_min) / angle_increment);  
        
        
        


        return range_data[i];
    }

    double get_error(const float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        double a = get_range(range_data, 45);
        double b = get_range(range_data, 90);

        double alpha = std::atan2(a*std::cos(theta) - b, a * std::sin(theta));

        double D_t = b * std::cos(alpha);
        double D_t1 = D_t + L * std::sin(alpha);
        error = dist - D_t1;
        
        return error;
    }

    void pid_control(double error)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            
            velocity: desired velocity -> this was removed since I couldn't fully understand why we needed this.

        Returns:
            None
        */

        double derivative = (error - prev_error) / 0.004; // 1/250 * 60 = 0.004 
        double new_integral = integral + error * 0.004;
        prev_error = error;
        integral = new_integral;
        double angle = 0.0;

        
        angle = -(kp * error + ki * new_integral + kd * derivative); 
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        double deg_angle = angle * 180 / M_PI;

        if (std::abs(deg_angle) < 10) {
            drive_msg.drive.speed = 4.0;
        } else if (std::abs(deg_angle) < 20) {
            drive_msg.drive.speed = 2.0;
        } else {
            drive_msg.drive.speed = 0.5;
        }
        
        drive_msg.header.stamp = this->get_clock()->now();
        pub_drive_->publish(drive_msg);
        
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        auto & range_data = scan_msg->ranges;
        angle_min = scan_msg->angle_min;
        angle_increment = scan_msg->angle_increment;

        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        
        double error = get_error(range_data.data(), 1.0); // TODO: replace with error calculated by get_error()
        pid_control(error);

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
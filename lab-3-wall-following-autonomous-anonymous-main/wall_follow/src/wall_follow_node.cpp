#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <typeinfo>
#include <cmath>
#include <stdio.h> // #include packages for sleep(int time_in_seconds)
#include <time.h> // #include packages for sleep(int time_in_seconds)
#include <unistd.h> // #include packages for sleep(int time_in_seconds)
#define _USE_MATH_DEFINES

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        this->publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->drive_topic, 10);
        this->subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));

    }

private:
    // PID CONTROL PARAMS
    // double kp = 0.5;
    // double kd = 0.01;
    // double ki = 0.0;
    double kp = 1.115;
    double kd = 0.08;
    double ki = 0.0000015;
    double L = 0.5; // Lookahead distance (meters), Jason added this
    
    double desired_dist = 0.9; // In meters, from the wall

    double car_width = 0.21; // meters, via range readings before and during wall collision
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    //other variables
    double delta_time = 0.0;
    double velocity = 0.0;
    
    // double pi = 3.14;
    // We can just use global constant M_PI imported from _USE_MATH_DEFINES

    //we might need to change it to seconds
    double prev_tmoment = this->get_clock()->now().seconds();

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan;

    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the cdLiDAR
            angle: between angle_min and angle_max of the LiDAR
            EDIT angle: between beam a and beam b from LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        float b = range_data[0];
        float a = range_data[1];
        
        double numer = a * cos(angle) - b;
        double denom = a * sin(angle);
        double alpha = atan(numer / denom);

        float D_t = b * cos(alpha);
        // std::cout<<"a: "<<a<<"; b: "<<b<<"; Alpha (deg): "<<alpha*(180.0/M_PI)<<std::endl;

        double D_tp1 = D_t + this->L * sin(alpha);

        return D_tp1;
    }

    // Jason changes arg float* range_data to float range
    double get_error(float range, double dist)
    {      
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */
        double error = range - dist;
        return error;

    }

    void pid_control(double error, bool is_dead_end, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            is_dead_end: if the car is running into a dead
            velocity: desired velocity

        Returns:
            None
        */

        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        double tmoment = this->get_clock()->now().seconds();
        delta_time = tmoment - prev_tmoment;
        integral += prev_error * delta_time;
        angle = (kp * error + kd * (error - prev_error) / delta_time + ki * integral);
        prev_tmoment = tmoment;

        if (abs(angle) > 20.0 / 180.0 * M_PI) {
            drive_msg.drive.speed = 1.5;
        } else if (abs(angle) > 10.0 / 180.0 * M_PI) {
            drive_msg.drive.speed = 1.75;
        } else {
            drive_msg.drive.speed = 2.0;
        }

        drive_msg.drive.steering_angle = angle;
        // drive_msg.drive.speed = 2.0;

        // If dead end, turn right
        if (is_dead_end) {
            drive_msg.drive.steering_angle = -1.5;
            drive_msg.drive.speed = 1.5;
        }

        prev_error = error;

        
        // Jason commented this out for debugging to just use teleop
        // And print what calculated distances are
        publisher_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        
        // Extacting variables from scan_msg
        // ranges = scan_msg->ranges; Unsure what type to use here b/c array
        float angle_max = scan_msg->angle_max;
        float angle_min = scan_msg->angle_min;
        float angle_range = angle_max - angle_min;
        float angle_increment = scan_msg->angle_increment;
        float time_increment = scan_msg->time_increment;
        float scan_time = scan_msg->scan_time;
        const std::vector<float> ranges = scan_msg->ranges;



        // Get size of array, copied code structure from something online
        int num_readings = 1080;
        
        // Another option
        // int num_readings = sizeof(ranges)/sizeof(ranges[0]);
        
        // Another option to get array size, if sizeof() doesn't work
        // std:: namespace might be unnecessary
        // float angle_range = angle_max - angle_min;
        // int num_readings = (int) std::round( angle_range / angle_increment ) + 1
        
        // Buggy first attempt
        // int b_index = std::round(num_readings / 2.0);

        // Corrected index calcuation
        int b_index = (int)round(1080.0*(M_PI_2-angle_min)/angle_range);

        // Angle (radians) between LiDAR beams, constrained between 0 and 70
        float theta_close = 35 * (M_PI / 180.0);

        // I did not see any "mid_index", do you mean "a_index"??
        int a_index_close = b_index - (int)round(1080.0*(theta_close/angle_range));
        float b_close = ranges[b_index];
        float a_close = ranges[a_index_close];
        float range_data_close[2];
        range_data_close[0] = b_close;
        range_data_close[1] = a_close;

        // Get two lidar readings at forward left and forward right
        float theta_front_1 = 45 * (M_PI / 180.0);
        int a_index_front_1 = b_index - (int)round(1080.0*(theta_front_1/angle_range));
        float range_front_1 = ranges[a_index_front_1];

        float theta_front_2 = 135 * (M_PI / 180.0);
        int a_index_front_2 = b_index - (int)round(1080.0*(theta_front_2/angle_range));
        float range_front_2 = ranges[a_index_front_2];

        float range_diff = range_front_2 - range_front_1;
        bool is_dead_end = false;

        if (range_diff > 1 && range_front_1 < 3) {
            std::cout << "Range front 1(m): " << range_front_1 <<  std::endl;
            std::cout << "Range front 2(m): " << range_front_2 <<  std::endl;
            is_dead_end = true;
        }
        else {
            is_dead_end = false;
        }


        double range_close = get_range(range_data_close, theta_close);
        double error = get_error(range_close, this->desired_dist);

        // sleep(500)

        // TODO Get velocity from pid_prev maybe?
        // Unsure how to set this desired velocity, or if it even matters since unused in
        // pid_control() function but rather reset in pid_control based on steering_angle
        pid_control(error, is_dead_end, velocity);


        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.
        Args:
            msg: Incoming LaserScan message
        Returns:
            None
        */

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <typeinfo>
// CHECK: include needed ROS msg type headers and libraries
#define _USE_MATH_DEFINES
/// CHECK: include needed ROS msg type headers and libraries

using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // std::cout << "First line of ReactiveFollowGap() constructor" << std::endl;
        /// TODO: create ROS subscribers and publishers
        this->publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->drive_topic, 10);
        this->subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Reactive Node Initialized");
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan;
    double prev_angle = 0.0;
    double integral = 0.0;
    double kp = 0.5;
    double kd = 0.3;
    double ki = 0.0000015;
    int window = 20;
    float pre_threshold = 5.0;
    float post_threshold = 3.0;
    double car_width = 0.2; // In meters, estimated
    double car_radius = car_width / 2;
    double prev_tmoment = this->get_clock()->now().seconds();
    double delta_time = 0.0;


    void preprocess_lidar(std::vector<std::vector<float>>& new_ranges, std::vector<float>& ranges, float angle_min, float angle_increment, float range_max)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        float currAngle = angle_min;
        for(int i = 0; i < ranges.size(); i++){
            int neighborhoodSize = 0;
            float neighborhoodSum = 0;
            if (i < 900 && i >= 180){
                for(int j = i - window; j <= i + window; j++){
                    if(j >= 0 && j < ranges.size()){
                        if(ranges[j] <= pre_threshold){
                            neighborhoodSize++;
                            neighborhoodSum += ranges[j];
                        }
                        // neighborhoodSize++;
                        // neighborhoodSum += std::min(3.f, ranges[j]);
                    }
                }
                if(neighborhoodSize != 0){
                    new_ranges.push_back(std::vector<float>{neighborhoodSum / neighborhoodSize, currAngle, i});
                }
            }
            currAngle += angle_increment;
        }
    }

    int* zeros_near_closest_point(std::vector<std::vector<float>>& ranges)
    {   
        
        // Find closest point
        int minIdx = 0;
        float minDist = ranges[0][0];
        for(int i = 0; i < ranges.size(); i++) {
            if(ranges[i][1] < 1.5 && ranges[i][1] > -1.5){
                if(ranges[i][0] < minDist){
                    minIdx = i;
                    minDist = ranges[i][0];
                }
            }
        }
        // std::cout << "MIN_IDX: " << minIdx << std::endl;
        // std::cout << "MIN_DIST: " << minDist << std::endl;


        // Set its whole neighborhood to 0

        double theta = abs(std::atan2(car_radius, minDist));
        // double theta = abs(std::asin(car_radius / minDist));

        ranges[minIdx][0] = 0.0;

        int leftIdx = minIdx - 1, rightIdx = minIdx + 1;
        while(leftIdx >= 0){
            float angle = abs(ranges[leftIdx][1] - ranges[minIdx][1]);
            // double distance = ranges[leftIdx][0] * ranges[leftIdx][0] + ranges[minIdx][0] * ranges[minIdx][0] - 2 * std::cos(angle);

            // if(distance > car_radius){
            //     break;
            // }

            if(angle > theta){
                break;
            }

            ranges[leftIdx--][0] = 0.0;
        }

        while(rightIdx < ranges.size()){
            float angle = abs(ranges[rightIdx][1] - ranges[minIdx][1]);
            // double distance = ranges[rightIdx][0] * ranges[rightIdx][0] + ranges[minIdx][0] * ranges[minIdx][0] - 2 * std::cos(angle);

            // if(distance > car_radius){
            //     break;
            // }

            if(angle > theta){
                break;
            }

            ranges[rightIdx++][0] = 0.0;
        }

        // Return indices bounding this section of zeros
        int* zero_ind_bounds = new int[2];
        zero_ind_bounds[0] = leftIdx + 1;
        zero_ind_bounds[1] = rightIdx - 1;
        // std::cout << "LEFT: " << zero_ind_bounds[0] << std::endl;
        // std::cout << "RIGHT: " << zero_ind_bounds[1] << std::endl;

        return zero_ind_bounds;
    }


    int find_max_gap_better(std::vector<std::vector<float>>& ranges) {

        unsigned int maxGapRightBound = ranges.size()/2;
        unsigned int maxGapLeftBound = ranges.size()/2;
        unsigned int currGapRightBound = ranges.size()/2;
        unsigned int currGapLeftBound = ranges.size()/2;
        bool gap = false;
        //RCLCPP_INFO(this->get_logger(), "Entered function: ");

        for(unsigned int i = 0; i < ranges.size(); i++) {
            
            if(ranges[i][0] >= post_threshold){
                //RCLCPP_INFO(this->get_logger(), "Angle: " + std::to_string(gap));
                //ranges[i][0] = post_threshold;
                currGapRightBound = i;
                if(!gap) {
                    currGapLeftBound = i;
                }
                if (currGapRightBound - currGapLeftBound > maxGapRightBound - maxGapLeftBound) {
                    maxGapRightBound = currGapRightBound;
                    maxGapLeftBound = currGapLeftBound;
                }
                gap = true;
            } else {
                gap = false;
            }
            
        }
        int maxGapCenter = (maxGapRightBound + maxGapLeftBound) / 2;
        return maxGapCenter;

    }


    int* find_max_gap(std::vector<std::vector<float>>& ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        int zero_ind_min = indice[0];
        int zero_ind_max = indice[1];
        int* max_gap = new int[2];

        float zero_ind_avg = (zero_ind_min + zero_ind_max) / 2.0;
        if (ranges[zero_ind_avg][1] <= 0.0){
            max_gap[0] = zero_ind_max + 1;
            max_gap[1] = ranges.size() - 1;
        } else {
            max_gap[0] = 0;
            max_gap[1] = zero_ind_min - 1;
        }

        // max_gap[0] = 0;
        // max_gap[1] = ranges.size()-1;

        return max_gap;
    }

    int naively_find_best_point_index(std::vector<std::vector<float>>& ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        int start_i = indice[0];
        int end_i = indice[1];

        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        // Find closest point
        int maxIdx = start_i;
        float maxDist = ranges[maxIdx][0];
        for(int i = start_i; i <= end_i && i < ranges.size(); i++) {
            if (ranges[i][0] > maxDist && ranges[i][1] < 0.5 * M_PI && ranges[i][1] > -0.5 * M_PI){
                maxIdx = i;
                maxDist = ranges[i][0];
            }
        }

        return maxIdx;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        /// TODO:
        // Find closest point to LiDAR
        // Eliminate all points inside 'bubble' (set them to zero) 
        // Find max length gap 
        // Find the best point in the gap 
        // Publish Drive message
        //auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        float angle_max = scan_msg->angle_max;
        float angle_min = scan_msg->angle_min;
        float angle_range = angle_max - angle_min;
        float angle_increment = scan_msg->angle_increment;
        float time_increment = scan_msg->time_increment;
        float scan_time = scan_msg->scan_time;
        float range_max = scan_msg->range_max;
        std::vector<float> old_ranges = scan_msg->ranges;
        std::vector<std::vector<float>> ranges;
//        RCLCPP_INFO(this->get_logger(), "In lidar callback");

        preprocess_lidar(ranges, old_ranges, angle_min, angle_increment, range_max);
        // std::cout << "ANGLE LEFT" << ranges[0][2] << std::endl;
        // std::cout << "DIST LEFT" << ranges[0][0] << std::endl;
        // std::cout << "ANGLE RIGHT" << ranges[ranges.size()-1][2] << std::endl;
        // std::cout << "DIST RIGHT" << ranges[ranges.size()-1][0] << std::endl;

        float disparity;
        for(int i = ranges.size()-1; i >= 1; i--) {
            disparity = ranges[i][0] - ranges[i-1][0];
            if (disparity > 0.15 && i > 270 && i < 810) {
                std::cout << "GAP: " << ranges[i][1] << std::endl;
                for (int j=0; j<40; j++){
                    if (i+j < ranges.size()-1){
                        ranges[i+j][0] = ranges[i-1][0];
                    } 
                }
            }
        }
        

        for(int i = 0; i < ranges.size()-1; i++) {
            disparity = ranges[i][0] - ranges[i+1][0];
            if (disparity > 0.15 && i > 270 && i < 810) {
                std::cout << "GAP: " << ranges[i][1] << std::endl;
                for (int j=0; j<40; j++){
                    if (i-j >= 0){
                        ranges[i-j][0] = ranges[i+1][0];
                    } 
                }
            }
        }

        // int bubble_radius = 10;
        // for(int i = 0; i < ranges.size()-1; i++) {
        //     disparity = abs(ranges[i][0] - ranges[i+1][0]);
        //     if (disparity > 0.2 && ranges[i][0] != 0) {
        //         for (int j=-bubble_radius; j<bubble_radius; j++){
        //             if (i+j >= 0 && i+j < ranges.size()-1){
        //                 ranges[i+j][0] = 0.0;
        //             } 
        //         }
        //     }
        // }

        int* zero_ind_bounds = zeros_near_closest_point(ranges);

        int* max_gap_ind_bounds = find_max_gap(ranges, zero_ind_bounds);

        int best_point_ind = naively_find_best_point_index(ranges, max_gap_ind_bounds);
        //int best_point_ind = find_max_gap_better(ranges);
        
        float max_dist = ranges[best_point_ind][0];
        float angle = ranges[best_point_ind][1];
        int array_length = ranges.size();
        int left_idx = 0.75 * array_length;
        int right_idx = 0.25 * array_length;
        // std::cout << "ANGLE RIGHT" << ranges[right_idx][1] << std::endl;
        // std::cout << "DIST RIGHT" << ranges[right_idx][0] << std::endl;
        // std::cout << "ANGLE LEFT" << ranges[left_idx][1] << std::endl;
        // std::cout << "DIST LEFT" << ranges[left_idx][0] << std::endl;
        if ((ranges[right_idx][0] < 0.35 && ranges[right_idx][0] > 0) || (ranges[0][0] < 0.2 && ranges[0][0] > 0)) {
            if (angle < 0){
                angle += 0.4;
                std::cout << "RIGHT OBS" << std::endl;
            } 
        } 
        
        if ((ranges[left_idx][0] < 0.35 && ranges[left_idx][0] > 0) || (ranges[array_length-1][0] < 0.2 && ranges[array_length-1][0] > 0)) {
            if (angle > 0){
                angle -= 0.4;
                std::cout << "LEFT OBS" << std::endl;
            }
        }

        if (angle < -0.7) {
            angle -= 0.7;
            std::cout << "TURNING RIGHT" << std::endl;
        } 
        
        if (angle > 0.7) {
            angle += 0.7;
            std::cout << "TURNING LEFT" << std::endl;
        }
        std::cout << "Best point ind: " << best_point_ind << std::endl;
        std::cout << "Max Dist: " << max_dist << std::endl;
        
        RCLCPP_INFO(this->get_logger(), "Angle: " + std::to_string(angle));
        double angle_control = 0.0;
        double tmoment = this->get_clock()->now().seconds();
        delta_time = tmoment - prev_tmoment;
        integral += prev_angle * delta_time;
        angle_control = (kp * angle + kd * (angle - prev_angle) / delta_time + ki * integral);
        prev_tmoment = tmoment;
        prev_angle = angle;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle_control;

        double speed = 0.5;
        if (abs(angle_control) > 30.0 / 180.0 * M_PI) {	
            drive_msg.drive.speed = 0.3 * speed;	
        } else if (abs(angle_control) > 20.0 / 180.0 * M_PI) {	
            drive_msg.drive.speed = 0.5 * speed;	
        } else if (abs(angle_control) > 10.0 / 180.0 * M_PI) {	
            drive_msg.drive.speed = 0.8 * speed;	
        } else if (abs(angle_control) > 5.0 / 180.0 * M_PI) {	
            drive_msg.drive.speed = 1.2 * speed;	
        } else {	
            if (max_dist > 5.0){	
                drive_msg.drive.speed = 5.0 * speed;	
            }	
            else {	
                drive_msg.drive.speed = max_dist / 5.0 * 1.5 * speed;	
            }	
        }

        //std::cout << "Speed: " << drive_msg.drive.speed << std::endl;
        publisher_->publish(drive_msg);
    }
};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>

// #include "rrt/rrt.h" // This might not work in PP package

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;
using std::placeholders::_1;

class PurePursuit : public rclcpp::Node
{

private:
    int idx = 0;

    double kp = 0.6;
    double kd = 0.0;
    double ki = 0.0;
    // this->ki = 0.00000015;
    double prevAngle = 0.0;
    double integral = 0.0;
    double lastTime = now().seconds();
    double speed = 6.5;
    double cornerKpCoeff = 2.0;

    double maxSteeringAngle = M_PI_4;
    double lookAheadDistance = 1.75; // 1.75;
    bool sim = true;
    bool opp = true;
    bool obstacle_detected = false;

    // Only needed for hard coded corners
    // int corner_1 = 493;
    // int corner_2 = 571;
    // int corner_3 = 180;
    // int corner_4 = 264;
    // int before_corner = 0; 
    // int after_corner = 0;


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr opp_drive_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_pub_;
    vector<geometry_msgs::msg::PoseWithCovarianceStamped> initposes;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // double goalX, goalY, carX, carY, goalX_global, goalY_global, carYaw;
    vector<vector<double>> waypoints;
    double global_carX, global_carY, global_carYaw;

    // Corner indices for test2.csv
    // vector<vector<int>> corners{{ 110, 146 }, 
    //                             { 185, 229 }, 
    //                             { 415, 455 }, 
    //                             { 497, 546 }};

    // Corner indices for new.csv
    // vector<vector<int>> corners{{ corner_1 - before_corner, corner_1 + after_corner}, 
    //                             { corner_2 - before_corner, corner_2 + after_corner}, 
    //                             { corner_3 - before_corner, corner_3 + after_corner}, 
    //                             { corner_4 - before_corner, corner_4 + after_corner}};


    int counter = 0;
    double lastUpdateTime;
    vector<vector<double>> globalPath;
    int deadman_counter = 0;


public:
    // Destructor of the PurePursuit class
     ~PurePursuit() {
        // Do something in here, free up used memory, print message, etc.
        RCLCPP_INFO(rclcpp::get_logger("PurePursuit"), "%s\n", "PurePursuit shutting down");
    }

    // Constructor of the PurePursuit class
    PurePursuit(): rclcpp::Node("pure_pursuit_node") {

        cout << "PurePursuit() node created" << endl;
        // ROS publishers
        // TODO: create publishers for the the drive topic, and other topics you might need
        // string occ_topic = "/occ_map";

        this->idx = 0;
        string drive_topic = "/drive";
        string joy_topic = "/joy";

        
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        // occ_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occ_topic, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_goal", 10);
        // path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rrt_path", 10);
        waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);

        initpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        
        // ROS subscribers
        // TODO: create subscribers as you need
        string pose_topic, file;
        if (sim) {
            pose_topic = "/ego_racecar/odom";
            file = "/home/alan/ESE6150/final-race-autonomous-anonymous/race3_optim.csv";
            file = "/home/alan/ESE6150/final-race-autonomous-anonymous/raceline_optim/race3_trim_v2.csv";
        } else {
            pose_topic = "/pf/pose/odom";
            file = "/home/nvidia/f1tenth_ws/src/final-race-autonomous-anonymous/race3_optim.csv";
            file = "/home/nvidia/f1tenth_ws/src/final-race-autonomous-anonymous/raceline_optim/raceline_opt2.csv";
            file = "/home/nvidia/f1tenth_ws/src/final-race-autonomous-anonymous/raceline_optim/race3_trim_v2.csv";
        }

        if (opp) {
            string opp_pose_topic = "/opp_racecar/odom";
            string opp_drive_topic = "/opp_drive";
            opp_drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(opp_drive_topic, 10);
            // opp_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            // opp_pose_topic, 10, std::bind(&PurePursuit::opp_pose_callback, this, std::placeholders::_1));
        }

        string scan_topic = "/scan";

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, 10, std::bind(&PurePursuit::joy_callback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 1, std::bind(&PurePursuit::scan_callback, this, std::placeholders::_1));

        // TODO: create a occupancy grid
        // mapSize = vector<double>{6, 5};
        // idxSize = vector<int>{int(mapSize[0] / resolution), int(mapSize[1] / resolution)};
        // occ = vector<signed char>(idxSize[0] * idxSize[1], 0);
        // publishMsg = true;

        // Initialize waypoints
        initWaypoints(file);

        pubWaypoints();

        // Initialize Random Generator
        // this->x_dist = std::uniform_real_distribution<>(0, mapSize[0]);
        // this->y_dist = std::uniform_real_distribution<> (-mapSize[1] / 2, mapSize[1] / 2);

        // this->radius = 0.1;
        // this->goalThresh = 0.05;

        this->lastUpdateTime = now().seconds();

        RCLCPP_INFO(rclcpp::get_logger("PurePursuit"), "%s\n", "Created new PurePursuit Object.");
    }

    void initWaypoints(const string fileName){
        cout << "initWaypoints() running" << endl;
        ifstream file(fileName);
        string value;
        string line;
        while(getline(file, line)){
            stringstream s(line);

            vector<double> row;
            while(getline(s, value, ',')){
                row.push_back(stod(value));
            }

            this->waypoints.push_back(row);
        }
	    cout << "Initialize Waypoints: " << this->waypoints.size() << endl;
        cout << this->waypoints[147][0] << ", " << this->waypoints[147][1] << endl;
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
        // The pose callback when subscribed to particle filter's inferred pose
        // Args:
        //    pose_msg (*PoseStamped): pointer to the incoming pose message
        // Returns:

        if (this->sim) {
            if (this->counter < 5) {
                this->counter++;
                return;
            } else {
                this->counter = 0;
            }
        }

       	// cout << "POSE CALLBACK" << endl; 

        auto pose = pose_msg->pose.pose;
        auto p = pose.position;
        auto o = pose.orientation;

        double carX = p.x;
        double carY = p.y;
        double carYaw = std::atan2( 2.0 * (o.w * o.z + o.x * o.y),
                                    1.0 - 2.0 * (o.y * o.y + o.z * o.z));

        this->global_carX = carX;
        this->global_carY = carY;
        this->global_carYaw = carYaw;

        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        vector<double> local_goal;
        double brakeCoeff = 1.0;
        double goalX_global = -1;
        double goalY_global = -1;
        bool foundGoal = false;
        for(int i=0; i < this->waypoints.size(); i++){
            auto point = this->waypoints[i];
            double dx = point[0] - carX;
            double dy = point[1] - carY;
            double dist = sqrt(dx * dx + dy * dy);

            if(abs(dist - this->lookAheadDistance) > 0.1)
                continue;

            double goalX = cos(yaw) * dx + sin(yaw) * dy;
            double goalY = -sin(yaw) * dx + cos(yaw) * dy;

            if(abs(std::atan(goalY / goalX)) < M_PI_4 * 1.5 && goalX > 0){
                goalX_global = point[0];
                goalY_global = point[1];
                cout << "Found Goal: " << i << endl;
                foundGoal = true;
                // brakeCoeff = this->brake(i);
                break;
            }
        }

        if(!foundGoal){
            cout << "ERROR: Found No Goal" << endl;
            return;
        }

        if(goalX_global == -1.0 || goalY_global == -1.0){
            cout << "ERROR: goal = -1" << endl;
            return;
        }

        //this->pubMarker(goalX_global, goalY_global, "map");

        double dx = goalX_global - carX, dy = goalY_global - carY;
        double goalX = dx * cos(-carYaw) - dy * sin(-carYaw);
        double goalY = dx * sin(-carYaw) + dy * cos(-carYaw);

        if (this->obstacle_detected) {
            goalY -= 0.7;
        }

        local_goal.push_back(goalX);
        local_goal.push_back(goalY);

        this->lastUpdateTime = now().seconds();

        // rrt_goal = this->globalToLocal(rrt_goal[0], rrt_goal[1], carX, carY, yaw);
        this->deadman_counter += 1;
        bool opp_drive = false;
        this->publishDriveMsg(local_goal, brakeCoeff, opp_drive);

    }

    void opp_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {

        if (this->sim) {
            if (this->counter < 5) {
                this->counter++;
                return;
            } else {
                this->counter = 0;
            }
        }

       	// cout << "POSE CALLBACK" << endl; 

        auto pose = pose_msg->pose.pose;
        auto p = pose.position;
        auto o = pose.orientation;

        double carX = p.x;
        double carY = p.y;
        double carYaw = std::atan2( 2.0 * (o.w * o.z + o.x * o.y),
                                    1.0 - 2.0 * (o.y * o.y + o.z * o.z));

        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        vector<double> local_goal;
        double brakeCoeff = 1.0;
        double goalX_global = -1;
        double goalY_global = -1;
        bool foundGoal = false;
        for(int i=0; i < this->waypoints.size(); i++){
            auto point = this->waypoints[i];
            double dx = point[0] - carX;
            double dy = point[1] - carY;
            double dist = sqrt(dx * dx + dy * dy);

            if(abs(dist - this->lookAheadDistance) > 0.1)
                continue;

            double goalX = cos(yaw) * dx + sin(yaw) * dy;
            double goalY = -sin(yaw) * dx + cos(yaw) * dy;

            if(abs(std::atan(goalY / goalX)) < M_PI_4 * 1.5 && goalX > 0){
                goalX_global = point[0];
                goalY_global = point[1];
                foundGoal = true;
                // brakeCoeff = this->brake(i);
                break;
            }
        }

        if(!foundGoal){
            cout << "ERROR: Found No Goal" << endl;
            return;
        }

        if(goalX_global == -1.0 || goalY_global == -1.0){
            cout << "ERROR: goal = -1" << endl;
            return;
        }

        //this->pubMarker(goalX_global, goalY_global, "map");

        double dx = goalX_global - carX, dy = goalY_global - carY;
        double goalX = dx * cos(-carYaw) - dy * sin(-carYaw);
        double goalY = dx * sin(-carYaw) + dy * cos(-carYaw);


        local_goal.push_back(goalX);
        local_goal.push_back(goalY);

        this->lastUpdateTime = now().seconds();
        bool opp_drive = true;
        this->publishDriveMsg(local_goal, brakeCoeff, opp_drive);
        // path found as Path message

    }

    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg) {
        // The joy callback when subscribed to joy topic
        // The RRT main loop happens here
        // Args:
        //    joy_msg (*Joy): pointer to the incoming joy message
        // Returns:
        //
        if (joy_msg->buttons[5] == 1) {
           // cout << "DEADMAN PRESSED" << endl;
            this->deadman_counter = 0;
        }

        return;
        
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();


        pose.pose.pose.orientation.x = 0.0;
        pose.pose.pose.orientation.y = 0.0;
        pose.pose.pose.orientation.z = 0.0;
        pose.pose.pose.orientation.w = 1.0;

        for(int i = 0; i < 35; i++){
            pose.pose.covariance[i] = 0.0;
        }
        pose.pose.covariance[0] = 0.25;
        pose.pose.covariance[7] = 0.25;
        pose.pose.covariance[35] = 0.06;

        if(joy_msg->buttons[5] == 1){
            pose.pose.pose.position.x = 0.8;
            pose.pose.pose.position.y = 0.55;
            pose.pose.pose.position.z = 0.0;

            this->initpose_pub_->publish(pose); 
        }
        else if(joy_msg->buttons[4] == 1){
            pose.pose.pose.position.x = 3.9;
            pose.pose.pose.position.y = 4.5;
            pose.pose.pose.position.z = 0.0;

            this->initpose_pub_->publish(pose); 
        }
        else if(joy_msg->buttons[4] == 1){
            pose.pose.pose.position.x = 5.3;
            pose.pose.pose.position.y = 8.2;
            pose.pose.pose.position.z = 0.0;

            this->initpose_pub_->publish(pose); 
        }
        else if(joy_msg->buttons[4] == 1){
            pose.pose.pose.position.x = 8.2;
            pose.pose.pose.position.y = 3.2;
            pose.pose.pose.position.z = 0.0;

            this->initpose_pub_->publish(pose); 
        }
    }

    void pubWaypoints() {
        cout << "Publishing Waypoints" << endl;
        int waypoint_size = this->waypoints.size();
        visualization_msgs::msg::MarkerArray waypoint_markers;
        for (int i = 0; i < waypoint_size-1; i++){
            vector<double> point = this->waypoints[i];
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now();
            marker.id = this->idx++;

            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = 0;

            marker.pose.position.x = point[0];
            marker.pose.position.y = point[1];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            waypoint_markers.markers.push_back(marker);
        }

        this->waypoints_pub_->publish(waypoint_markers);
    }
    void pubMarker(double x, double y, string frame_id){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = now();
        marker.id = this->idx++;

        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = 0;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // cout << "Publishing marker" << endl;
        this->marker_pub_->publish(marker);
    }

    void publishDriveMsg(vector<double> goal, double brakeCoeff, bool opp_drive){
        if(goal.empty())
            return;

        double KP_Pursuit = 1.0;
        double curvature = 2.0 * goal[1] / pow(this->lookAheadDistance, 2);
        double steeringAngle = KP_Pursuit * curvature;


        //if (abs(brakeCoeff) < 0.6) {
        //    steeringAngle *= this->cornerKpCoeff;
        //}
        
        // steeringAngle *= (1 + curvature);

        double t = now().seconds();
        double dt = 1.0 * (t - this->lastTime);
        this->integral += this->prevAngle * dt;
        double angleControl = (this->kp * steeringAngle + 
                                this->kd * (steeringAngle - this->prevAngle) / dt +
                                this->ki * this->integral);
        this->lastTime = t;

        angleControl = min(this->maxSteeringAngle, max(-this->maxSteeringAngle, angleControl));
        this->prevAngle = angleControl;

        // float video_mult = 11.0 / 39;
        // speed *= video_mult;

        double new_speed = this->speed * abs(1 - abs(steeringAngle / 2));
        // cout << "Steering Control: " << steeringAngle << endl;
        // cout << "Speed: " << new_speed << endl;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angleControl;
        // if(abs(angleControl) > 20.0 / 180.0 * (22/7))
        //    drive_msg.drive.speed = float(0.75 * new_speed);
        //else if(abs(angleControl) > 10.0 / 180.0 * (22/7))
        //    drive_msg.drive.speed = float(0.875 * new_speed);
        //else
        //    drive_msg.drive.speed = float(1.0 * new_speed);
        drive_msg.drive.speed = new_speed;
        
        // this->deadman_counter++;
        // cout << this->deadman_counter << endl;
        // if (this->deadman_counter > 10) {
        //      drive_msg.drive.speed = 0.0;
        //      drive_msg.drive.steering_angle = 0.0;
        // } 
        if (!opp_drive) {
            cout << "here" << endl;
            this->drive_pub_->publish(drive_msg);
        }
        else {
            auto opp_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            opp_drive_msg.drive.steering_angle = angleControl;
            opp_drive_msg.drive.speed = new_speed * 0.7;
            this->opp_drive_pub_->publish(opp_drive_msg);
        }

    }

    vector<double> globalToLocal(double globalX, double globalY, double carX, double carY, double yaw){
        double dx = globalX - carX;
        double dy = globalY - carY;

        double x = dx * cos(-yaw) - dy * sin(-yaw);
        double y = dx * sin(-yaw) + dy * cos(-yaw);

        return vector<double>{x, y};
    }

    vector<double> localToGlobal(double localX, double localY, double carX, double carY, double yaw){
        double x = localX * cos(yaw) - localY * sin(yaw);
        double y = localX * sin(yaw) + localY * cos(yaw);

        return vector<double>{carX + x, carY + y};
    }

    // double brake(int index){
    //     double brakeCoeff = 1.0;
    //     for (auto corner: this->corners){
    //         if (index >= corner[0] && index <= corner[1]){
    //             double front = (corner[1] - (double)index) / (corner[1] - corner[0]);
    //             double back = ((double)index - corner[0]) / (corner[1] - corner[0]);
    //             // brakeCoeff -= front * back * 2.0;
	// 	brakeCoeff = 0.42;
    //             if (index < (corner[1] + corner[2]) / 2) {
    //                 brakeCoeff = -brakeCoeff;
    //             }
    //             break;
    //         }
    //     }
    //     return brakeCoeff;
    // }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan_ranges) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        /// TODO:
        // Find closest point to LiDAR
        // Eliminate all points inside 'bubble' (set them to zero) 
        // Find max length gap 
        // Find the best point in the gap 
        // Publish Drive message
        //auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        double obstacle_detection_range = this->lookAheadDistance;               
        double obstacle_detection_angle = M_PI / 6;          
        double overtaking_distance_threshold = 2.5;           
        double overtaking_lateral_distance_threshold = 0.25;
        double obstacle_distance_threshold = 0.1;
        bool obstacle_on_traj = false;
        // double angle = scan_msg->angle_min;
        // while (angle <= scan_msg->angle_max) {

        // }

        this->obstacle_detected = false;
        double min_obstacle_distance = 0.0;
        double carYaw = 0.0;
        vector<vector<double>> scan_waypoints = this->waypoints;

        for (int i = 0; i < laser_scan_ranges->ranges.size(); i++) {
            double angle = laser_scan_ranges->angle_min + i * laser_scan_ranges->angle_increment;
            double distance = laser_scan_ranges->ranges[i];
            double x = distance * cos(angle);
            double y = distance * sin(angle);
            vector<double> globalObs = this->localToGlobal(x, y, this->global_carX, this->global_carY, this->global_carYaw);
            // for(auto point : scan_waypoints){
            //     // double dx = point[0] - globalObs[0], dy = point[1] - globalObs[1];
            //     cout << "somethin" << endl;
            //     // double dist = sqrt(dx * dx + dy * dy);

            //     // // cout << dist << endl;

            //     // if(dist < obstacle_distance_threshold){
            //     //     obstacle_on_traj = true;
            //     //     // break;
            //     // }
            //     // cout << i << endl;
            // }


            // cout << "Here" << endl;
            
            if (distance > 0.0 && distance < obstacle_detection_range && abs(angle) < obstacle_detection_angle / 2) {
                this->obstacle_detected = true;
                if (min_obstacle_distance == 0.0 || distance < min_obstacle_distance) {
                    min_obstacle_distance = distance;
                }
            }
            if (this->obstacle_detected){
                cout << "OBS" << endl;
            }
        }

        // bool overtaking_needed = false;
        // if (obstacle_detected && min_obstacle_distance < overtaking_distance_threshold) {
        //     double obstacle_angle = atan2(min_obstacle_distance * sin(obstacle_detection_angle / 2), min_obstacle_distance * cos(obstacle_detection_angle / 2));
        //     double angle_difference = obstacle_angle - carYaw;
        //     double lateral_distance = min_obstacle_distance * sin(angle_difference);
        //     if (lateral_distance > -overtaking_lateral_distance_threshold && lateral_distance < overtaking_lateral_distance_threshold) {
        //         overtaking_needed = true;
        //     }
        // }
        //cout << "Overtaking: " << overtaking_needed << endl;
    }

};

int main(int argc, char **argv)
{
    
    cout << "First line of main()" << endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    cout << "Last line of main()" << endl;
    return 0;
}

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
#include "geometry_msgs/msg/pose_stamped.hpp"
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

    double kp = 0.25;
    double kd = 0.0;
    double ki = 0.0;
    // this->ki = 0.00000015;
    double prevAngle = 0.0;
    double integral = 0.0;
    double lastTime = now().seconds();
    double speed = 6.0;
    double cornerKpCoeff = 2.0;

    double maxSteeringAngle = M_PI_4;
    double lookAheadDistance = 2.0;

    int corner_1 = 473;
    int corner_2 = 550;
    int corner_3 = 170;
    int corner_4 = 246;
    int before_corner = 20; 
    int after_corner = 40;


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    //rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    //rclcpp::Subscription<sensor_msgs::msg::LaserScan> scan_sub_;

    double goalX, goalY;
    vector<vector<double>> waypoints;

    // Corner indices for test2.csv
    // vector<vector<int>> corners{{ 110, 146 }, 
    //                             { 185, 229 }, 
    //                             { 415, 455 }, 
    //                             { 497, 546 }};

    // Corner indices for new.csv
    vector<vector<int>> corners{{ corner_1 - before_corner, corner_1 + after_corner}, 
                                { corner_2 - before_corner, corner_2 + after_corner}, 
                                { corner_3 - before_corner, corner_3 + after_corner}, 
                                { corner_4 - before_corner, corner_4 + after_corner}};


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
        
        // ROS subscribers
        // TODO: create subscribers as you need
        string pose_topic = "/ego_racecar/odom";
        // string pose_topic = "/pf/pose/odom";
        string scan_topic = "/scan";

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
        //joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        //joy_topic, 10, std::bind(&PurePursuit::joy_callback, this, std::placeholders::_1));
        //scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //scan_topic, 1, std::bind(&scan_callback, this, std::placeholders::_1));

        // TODO: create a occupancy grid
        // mapSize = vector<double>{6, 5};
        // idxSize = vector<int>{int(mapSize[0] / resolution), int(mapSize[1] / resolution)};
        // occ = vector<signed char>(idxSize[0] * idxSize[1], 0);
        // publishMsg = true;

        // Initialize waypoints
        // string file = "teleop_waypoints_sim_1.csv";
        
        // string file = "test2.csv";
        string file = "despacito_new.csv";
        initWaypoints(file);

        pubWaypoints();

        // Initialize PID parameters
        // initPID();

        // Initialize Random Generator
        // this->x_dist = std::uniform_real_distribution<>(0, mapSize[0]);
        // this->y_dist = std::uniform_real_distribution<> (-mapSize[1] / 2, mapSize[1] / 2);

        // this->radius = 0.1;
        // this->goalThresh = 0.05;

        this->lastUpdateTime = now().seconds();

        RCLCPP_INFO(rclcpp::get_logger("PurePursuit"), "%s\n", "Created new PurePursuit Object.");
    }

    // void initPID(){
    //     double kp = 1.0;
    //     double kd = 0.0;
    //     double ki = 0.0;
    //     // this->ki = 0.00000015;
    //     double prevAngle = 0.0;
    //     double integral = 0.0;
    //     double lastTime = now().seconds();

    //     double maxSteeringAngle = M_PI_4;
    // }

    void initWaypoints(const string fileName){
        cout << "initWaypoints() running" << endl;
        ifstream file(fileName);
        string value;
        while(file.good()){
            string line;
            getline(file, line);

            stringstream s(line);

            vector<double> row;
            while(getline(s, value, ',')){
                row.push_back(stod(value));
            }

            this->waypoints.push_back(row);
        }
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
        // The pose callback when subscribed to particle filter's inferred pose
        // The RRT main loop happens here
        // Args:
        //    pose_msg (*PoseStamped): pointer to the incoming pose message
        // Returns:
        //
        if (this->counter < 5) {
            this->counter++;
            return;
        } else {
            this->counter = 0;
        }
       	// cout << "POSE CALLBACK" << endl; 
        auto pose = pose_msg->pose.pose;
        auto p = pose.position;
        auto o = pose.orientation;

        double carX = p.x, carY = p.y;
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
        double goalX_global = -1, goalY_global = -1;
        for(int i=0; i < this->waypoints.size(); i++){
            auto point = this->waypoints[i];
            double dx = point[0] - carX;
            double dy = point[1] - carY;
            double dist = sqrt(dx * dx + dy * dy);

            if(abs(dist - this->lookAheadDistance) > 0.1)
                continue;

            double goalX = cos(yaw) * dx + sin(yaw) * dy;
            double goalY = -sin(yaw) * dx + cos(yaw) * dy;

            if(abs(std::atan(goalX / goalY)) < M_PI_2 && goalX > 0){
                goalX_global = point[0];
                goalY_global = point[1];
                brakeCoeff = this->brake(i);
                break;
            }
        }
        
        this->pubMarker(goalX_global, goalY_global, "map");

        double dx = goalX_global - carX, dy = goalY_global - carY;
        this->goalX = dx * cos(-carYaw) - dy * sin(-carYaw);
        this->goalY = dx * sin(-carYaw) + dy * cos(-carYaw);

        local_goal.push_back(this->goalX);
        local_goal.push_back(this->goalY);

        this->lastUpdateTime = now().seconds();

        // rrt_goal = this->globalToLocal(rrt_goal[0], rrt_goal[1], carX, carY, yaw);
        this->deadman_counter += 1;
        this->publishDriveMsg(local_goal, brakeCoeff);
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
            cout << "DEADMAN PRESSED" << endl;
            this->deadman_counter = 0;
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
	    
	    if (i == this->corner_1 || i == this->corner_2 || i == this->corner_3 || i == this->corner_4) {
		    marker.color.r = 0.0;
		    marker.color.b = 1.0;
	    }
	    
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

    void publishDriveMsg(vector<double> goal, double brakeCoeff){

        double KP_Pursuit = 1.0;
        double curvature = 2.0 * goal[1] / pow(this->lookAheadDistance, 2);
        // cout << "publishDriveMsg() line 268 reached" << endl;

        double steeringAngle = KP_Pursuit * curvature;
        // cout << "publishDriveMsg() line 271 reached" << endl;


        //if (abs(brakeCoeff) < 0.6) {
        //    steeringAngle *= this->cornerKpCoeff;
        //}
        
        steeringAngle *= (1 + curvature);

        double t = now().seconds();
        double dt = 1.0 * (t - this->lastTime);

        // cout << "publishDriveMsg() line 277 reached" << endl;

        this->integral += this->prevAngle * dt;
        double angleControl = (this->kp * steeringAngle + 
                                this->kd * (steeringAngle - this->prevAngle) / dt +
                                this->ki * this->integral);
        this->lastTime = t;
        // cout << "publishDriveMsg() line 279 reached" << endl;

        angleControl = min(this->maxSteeringAngle, max(-this->maxSteeringAngle, angleControl));
        this->prevAngle = angleControl;

        // float video_mult = 11.0 / 39;
        // speed *= video_mult;

        double new_speed = this->speed * abs(brakeCoeff);
        cout << "Speed: " << new_speed << endl;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angleControl;
        // if(abs(angleControl) > 20.0 / 180.0 * (22/7))
        //    drive_msg.drive.speed = float(0.75 * new_speed);
        //else if(abs(angleControl) > 10.0 / 180.0 * (22/7))
        //    drive_msg.drive.speed = float(0.875 * new_speed);
        //else
        //    drive_msg.drive.speed = float(1.0 * new_speed);
        drive_msg.drive.speed = new_speed;
        
        this->deadman_counter++;
        // cout << this->deadman_counter << endl;
        //if (this->deadman_counter > 10) {
        //     drive_msg.drive.speed = 0.0;
        //     drive_msg.drive.steering_angle = 0.0;
        //} 
        this->drive_pub_->publish(drive_msg);

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

    double brake(int index){
        double brakeCoeff = 1.0;
        double lowCoeff = 0.5;
        double left = 0.33;
        double right = 0.66;

        for (auto corner: this->corners){
            if (index >= corner[0] && index <= corner[1]){
                int range = corner[1] - corner[0];
                double back = (corner[1] - (double)index) / range;
                double front = ((double)index - corner[0]) / range;
                // brakeCoeff -= front * back * 2.0;
                if (front < left) {
                    brakeCoeff = brakeCoeff - front / left * (brakeCoeff - lowCoeff);
                } else if (front < right) {
                    brakeCoeff = lowCoeff;
                } else {
                    brakeCoeff = brakeCoeff - back / (1 - right) * (brakeCoeff - lowCoeff);
                }
                break;
            }
        }
	cout << "Brake: " << brakeCoeff << endl;
        return brakeCoeff;
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

#include "fr_pkg/camera.hpp"

// Includes copied over from rrt.h from Lab 6
#include <iostream>
#include <fstream>
#include <istream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
// Includes copied over from rrt.h from Lab 6
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<RRT>()); // THIS IS OUTDATED, COPIED FROM LAB 6!
    rclcpp::shutdown();
    return 0;
}
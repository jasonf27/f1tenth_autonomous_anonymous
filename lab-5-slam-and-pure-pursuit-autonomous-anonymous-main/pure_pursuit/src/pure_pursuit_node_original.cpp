#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

// using namespace std;
// using std::placeholders::_1;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:
    std::string marker_points = "/marker";
    std::string drive_topic = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_1;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_2;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_3;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_1;


public:
    // PurePursuit() : Node("pure_pursuit_node_cpp_outdated")
    PurePursuit() : Node("pure_pursuit_node_original")
    {
        // TODO: create ROS subscribers and publishers
        std::cout << "C++ PurePursuit() class, node created" << std::endl;
        
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    {
        std::cout << "pose_callball() executes" << std::endl;
    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    std::cout << "main() run within C++ node" << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
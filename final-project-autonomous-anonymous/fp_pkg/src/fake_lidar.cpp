#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std;

class FakeLidarGenerator : public rclcpp::Node {

public:
	FakeLidarGenerator(): Node("Fake LiDAR Generator") {
		this->pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/fake_scan", 10);

		this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/binary_image", 10, std::bind(&FakeLidarGenerator::image_callback, this, _1));
    	
	}

private:

	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const{
		
	
	}

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<FakeLidarGenerator>();

  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

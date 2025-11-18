#include "rtcrobot_hmi_denso/RosClient.hpp"

RosClient::RosClient()
  : rclcpp::Node("hmi_client"), rtcrobot_core::NodeCore("hmi_client") {
  initialize();

  RCLCPP_INFO(this->get_logger(), "ROS Client initialized");
}

RosClient::~RosClient() {
  RCLCPP_INFO(this->get_logger(), "ROS Client destroyed");
}

#ifndef ROS_CLIENT_HPP
#define ROS_CLIENT_HPP

#include <cmath>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <rtc_nodecore/node_core.hpp>



class RosClient : public rclcpp::Node, public rtcrobot_core::NodeCore {
public:
  RosClient();
  ~RosClient();

private:
  void init_publishers();
  void init_subscribers();


};

#endif  // ROS_CLIENT_HPP
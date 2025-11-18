#ifndef ROS_CLIENT_HPP
#define ROS_CLIENT_HPP

#include <cmath>
#include <memory>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <rtc_nodecore/node_core.hpp>
#include <rtc_nodecore/signal_handler.hpp>
#include <rtcrobot_state_server/msg/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rtcrobot_map_manager/srv/load_map.hpp>
#include <cartographer_ros_msgs/msg/status_code.hpp>
#include <cartographer_ros_msgs/msg/status_response.hpp>

using std_srvs::srv::Trigger;

constexpr char kStateTopic[]                   = "/agv/local/state";
constexpr char kCmdVelTopic[]                  = "/cmd_vel_hmi";
constexpr char kServiceStartAutomaticMode[]    = "/agv/mode/start_automatic";
constexpr char kAddTagRelocationService[]      = "/add_tag_relocation";
constexpr char kRelocationService[]            = "/relocation";
constexpr char kLoadMapService[]               = "/load_map";
constexpr char kStartLocalizationServiceName[] = "/start_localization";

// TODO: Bạn cần sửa StateAGV type phù hợp với topic /agv/local/state
using StateAGV = rtcrobot_state_server::msg::State;
using Twist    = geometry_msgs::msg::Twist;
using LoadMap  = rtcrobot_map_manager::srv::LoadMap;

class RosClient : public rclcpp::Node, public rtcrobot_core::NodeCore {
public:
  RosClient();
  ~RosClient();

  StateAGV getStateAGV() const;

  void publishCmdVel(double linear_velocity, double angular_velocity);
  void setStart(std::function<void(std::string, bool)> callback = nullptr);
  void setStop();
  void setRelocation(std::function<void(std::string, bool)> callback = nullptr);
  void setSelectMap(const std::string& map_name);
  void setAddTag(std::function<void(std::string, bool)> callback = nullptr);
  void setReset();
  void setMaxVelocity(double max_velocity);

  std::string getReturnMessageOfService() const;

private:
  void init_publishers();
  void init_subscribers();
  void init_clients();

private:
  rclcpp::Client<Trigger>::SharedPtr client_start_automatic_mode_{nullptr};
  rclcpp::Client<Trigger>::SharedPtr client_add_tag_relocation_{nullptr};
  rclcpp::Client<Trigger>::SharedPtr client_relocation_{nullptr};
  rclcpp::Client<LoadMap>::SharedPtr client_load_map_{nullptr};

  rclcpp::Subscription<StateAGV>::SharedPtr sub_state_;
  rclcpp::Publisher<Twist>::SharedPtr       pub_cmd_vel_;
  StateAGV                                  last_state_agv_;
  double                                    max_linear_velocity_ = 0.5;

  std::string return_message_of_service_{""};
};

#endif  // ROS_CLIENT_HPP
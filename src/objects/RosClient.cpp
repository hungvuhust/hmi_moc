#include "rtcrobot_hmi_denso/RosClient.hpp"
#include <cartographer_ros_msgs/msg/detail/status_code__struct.hpp>

RosClient::RosClient()
  : rclcpp::Node("hmi_client"), rtcrobot_core::NodeCore("hmi_client") {
  initialize();
  init_publishers();
  init_subscribers();
  init_clients();
  RCLCPP_INFO(this->get_logger(), "ROS Client initialized");
}

RosClient::~RosClient() {
  RCLCPP_INFO(this->get_logger(), "ROS Client destroyed");
}

void RosClient::init_publishers() {
  RCLCPP_INFO(this->get_logger(), "Initializing publishers");
  pub_cmd_vel_ = create_publisher<Twist>(kCmdVelTopic, 10);
}

void RosClient::init_subscribers() {
  RCLCPP_INFO(this->get_logger(), "Initializing subscribers");
  sub_state_ =
    create_subscription<StateAGV>(kStateTopic,
                                  10,
                                  [this](const StateAGV::SharedPtr msg) {
                                    last_state_agv_ = *msg;
                                  });
}

void RosClient::init_clients() {
  RCLCPP_INFO(this->get_logger(), "Initializing clients");
  client_start_automatic_mode_ =
    create_client<Trigger>(kServiceStartAutomaticMode);
  client_add_tag_relocation_ = create_client<Trigger>(kAddTagRelocationService);
  client_relocation_         = create_client<Trigger>(kRelocationService);
}

StateAGV RosClient::getStateAGV() const {
  return last_state_agv_;
}

void RosClient::publishCmdVel(double linear_velocity, double angular_velocity) {
  if (!pub_cmd_vel_) {
    RCLCPP_WARN(this->get_logger(), "Cmd vel publisher not initialized");
    return;
  }

  Twist cmd_vel_msg;
  cmd_vel_msg.linear.x  = linear_velocity;
  cmd_vel_msg.angular.z = angular_velocity;
  pub_cmd_vel_->publish(cmd_vel_msg);
}

void RosClient::setStart(std::function<void(std::string, bool)> callback) {
  if (current_state_.paused and
      current_state_.operating_mode != vda5050_msgs::msg::State::MANUAL) {
    this->setPaused(false);
  }

  return_message_of_service_ = "";
  if (client_start_automatic_mode_ and
      current_state_.operating_mode ==
        vda5050_msgs::msg::State::SEMIAUTOMATIC) {
    // Send sync request to start automatic mode
    if (!client_start_automatic_mode_->wait_for_service(
          std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for service");
        return_message_of_service_ = "Interrupted while waiting for service";
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service not available, waiting again...");
    }
    auto request = std::make_shared<Trigger::Request>();
    auto response =
      client_start_automatic_mode_->async_send_request(request).get();
    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start automatic mode");
      return_message_of_service_ = "Failed to start automatic mode";
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Started automatic mode");
    return_message_of_service_ = "Started automatic mode successfully";
  } else {
    RCLCPP_ERROR(this->get_logger(), "Operating mode is not valid");
    return_message_of_service_ = "Operating mode is not valid";
  }
}

void RosClient::setStop() {
  if (!current_state_.paused) {
    this->setPaused(true);
  }
}

void RosClient::setReset() {
  if (current_state_.errors.size() > 0) {
    this->releaseAllErrors();
  }
}

void RosClient::setMaxVelocity(double max_velocity) {
  max_linear_velocity_ = max_velocity;
}

void RosClient::setAddTag(std::function<void(std::string, bool)> callback) {
  return_message_of_service_ = "";
  if (client_add_tag_relocation_ and current_state_.agv_position.map_id != "") {
    // Send sync request to add tag
    if (!client_add_tag_relocation_->wait_for_service(
          std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for service");
        return_message_of_service_ = "Interrupted while waiting for service";
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service not available, waiting again...");
    }
    auto request = std::make_shared<Trigger::Request>();
    auto response =
      client_add_tag_relocation_->async_send_request(request).get();
    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add tag");
      return_message_of_service_ = response->message;
    } else {
      RCLCPP_INFO(this->get_logger(), "Added tag");
      return_message_of_service_ = "Added tag successfully";
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Map ID is not valid");
    return_message_of_service_ = "Map ID is not valid";
  }
}

void RosClient::setRelocation(std::function<void(std::string, bool)> callback) {
  return_message_of_service_ = "";
  if (client_relocation_ and current_state_.agv_position.map_id != "") {
    // Send sync request to relocation
    if (!client_relocation_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for service");
        return;
      }
    }
    auto request  = std::make_shared<Trigger::Request>();
    auto response = client_relocation_->async_send_request(request).get();
    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to relocation");
      return_message_of_service_ = response->message;
    } else {
      RCLCPP_INFO(this->get_logger(), "Relocation");
      return_message_of_service_ = "Relocation successfully";
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Map ID is not valid");
    return_message_of_service_ = "Map ID is not valid";
  }
}

std::string RosClient::getReturnMessageOfService() const {
  return return_message_of_service_;
}

void RosClient::setSelectMap(const std::string& map_name) {
  return_message_of_service_ = "";
  if (client_load_map_ and map_name != "") {
    // Send sync request to load map
    if (!client_load_map_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for service");
        return_message_of_service_ = "Interrupted while waiting for service";
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service not available, waiting again...");
    }
  }
  auto request      = std::make_shared<LoadMap::Request>();
  request->map_name = map_name;
  auto response     = client_load_map_->async_send_request(request).get();
  if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map");
    return_message_of_service_ = response->status.message;
  } else {
    RCLCPP_INFO(this->get_logger(), "Loaded map");
    return_message_of_service_ = "Loaded map successfully";
  }
}

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

enum class RobotState
{
  IDLE,
  NAVIGATING_TO_PICKUP,
  PICKUP_ACTION,
  NAVIGATING_TO_DELIVERY,
  DROPOFF_ACTION,
  RETURNING_TO_DOCK
};

class MissionManager : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionManager();

private:
  // FSM + helpers
  void load_locations();
  void fsm_loop();
  void send_nav_goal(const YAML::Node & location);
  void publish_status(const std::string & text);

  // Nav2 callbacks
  void nav_result_callback(
    const GoalHandleNav::WrappedResult & result);

  // ROS interfaces
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  YAML::Node locations_;
  RobotState state_;
  bool goal_active_;
};

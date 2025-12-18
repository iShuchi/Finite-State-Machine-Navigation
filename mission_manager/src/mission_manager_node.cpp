#include "mission_manager/mission_manager_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>

MissionManager::MissionManager()
: Node("mission_manager"),
  state_(RobotState::IDLE),
  goal_active_(false)
{
  nav_client_ =
    rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

  status_pub_ =
    this->create_publisher<std_msgs::msg::String>(
      "/robot_status", 10);

  load_locations();

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MissionManager::fsm_loop, this));

  RCLCPP_INFO(this->get_logger(),
              "Mission Manager (C++) started");
}

void MissionManager::load_locations()
{
  const std::string pkg_path =
    ament_index_cpp::get_package_share_directory(
      "mission_manager");

  locations_ =
    YAML::LoadFile(pkg_path + "/config/locations.yaml");
}

void MissionManager::publish_status(
  const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
}

void MissionManager::send_nav_goal(
  const YAML::Node & location)
{
  if (goal_active_) {
    return;  // prevent goal spam
  }

  auto goal_msg = NavigateToPose::Goal();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  pose.pose.position.x = location["x"].as<double>();
  pose.pose.position.y = location["y"].as<double>();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, location["yaw"].as<double>());
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  goal_msg.pose = pose;

  nav_client_->wait_for_action_server();

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.result_callback =
    std::bind(&MissionManager::nav_result_callback,
              this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, options);
  goal_active_ = true;
}

void MissionManager::nav_result_callback(
  const GoalHandleNav::WrappedResult & result)
{
  goal_active_ = false;

  if (result.code !=
      rclcpp_action::ResultCode::SUCCEEDED)
  {
    publish_status(
      "Navigation failed → Returning to Dock");
    state_ = RobotState::RETURNING_TO_DOCK;
    send_nav_goal(locations_["dock_zone"]);
    return;
  }

  switch (state_)
  {
    case RobotState::NAVIGATING_TO_PICKUP:
      publish_status("Arrived at Pickup");
      state_ = RobotState::PICKUP_ACTION;
      break;

    case RobotState::NAVIGATING_TO_DELIVERY:
      publish_status("Arrived at Drop-off");
      state_ = RobotState::DROPOFF_ACTION;
      break;

    case RobotState::RETURNING_TO_DOCK:
      publish_status("Arrived at Dock → IDLE");
      state_ = RobotState::IDLE;
      break;

    default:
      break;
  }
}

void MissionManager::fsm_loop()
{
  switch (state_)
  {
    case RobotState::IDLE:
      publish_status("IDLE → Starting mission");
      state_ = RobotState::NAVIGATING_TO_PICKUP;
      send_nav_goal(locations_["pickup_zone"]);
      break;

    case RobotState::PICKUP_ACTION:
      publish_status("Pickup successful");
      state_ = RobotState::NAVIGATING_TO_DELIVERY;
      send_nav_goal(locations_["dropoff_zone"]);
      break;

    case RobotState::DROPOFF_ACTION:
      publish_status("Drop-off successful");
      state_ = RobotState::RETURNING_TO_DOCK;
      send_nav_goal(locations_["dock_zone"]);
      break;

    default:
      // waiting for Nav2 result
      break;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}

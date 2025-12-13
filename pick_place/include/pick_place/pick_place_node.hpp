
#ifndef PICK_PLACE__PICK_PLACE_NODE_HPP_
#define PICK_PLACE__PICK_PLACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <vector>
#include <algorithm>

namespace pick_place
{

/**
 * @brief A simple pick and place node demonstrating MoveIt planning and
 * interaction with Ignition Gazebo via ros_gz bridge. Implementation is
 * in pick_place_node.cpp.
 */
class PickPlaceNode : public rclcpp::Node
{
public:
  explicit PickPlaceNode();

private:
  void initializeMoveIt();
  void executePickPlaceSequence();

  bool moveToPose(double x, double y, double z, double roll, double pitch, double yaw);
  bool moveToPickApproach();
  bool moveToPickPose();
  bool liftObject();
  bool moveToPlaceApproach();
  bool moveToPlacePose();
  bool moveAway();

  bool attachObject();
  bool detachObject();

  // MoveIt objects
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  // Gazebo bridge service clients
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr attach_client_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr detach_client_;

  // Timer for initialization
  rclcpp::TimerBase::SharedPtr init_timer_;
  bool initialized_;
  bool object_attached_;

  // Joint state subscription and tracking
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  bool joint_states_received_ = false;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

  // Parameters and state
  std::vector<double> pick_pos_;
  std::vector<double> place_pos_;
  std::string object_name_;
  std::string planning_group_;
  std::string end_effector_link_;
  double planning_time_;
  int planning_attempts_;
  double velocity_scaling_;
  double acceleration_scaling_;
};

}  // namespace pick_place

#endif  // PICK_PLACE__PICK_PLACE_NODE_HPP_

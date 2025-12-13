// Implementation of the PickPlace node moved to a separate translation unit.
#include <rclcpp/rclcpp.hpp>
#include "pick_place/pick_place_node.hpp"

// Additional includes needed for implementation
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <chrono>

namespace pick_place
{

PickPlaceNode::PickPlaceNode()
  : Node("pick_place_cpp"), initialized_(false), object_attached_(false)
{
  this->declare_parameter<std::vector<double>>("pick_position", {0.6, 0.0, 0.025});
  this->declare_parameter<std::vector<double>>("place_position", {-0.5, 0.0, 0.025});
  this->declare_parameter<std::string>("object_name", "pick_object");
  this->declare_parameter<std::string>("planning_group", "ur5_manipulator");
  this->declare_parameter<std::string>("end_effector_link", "tool0");
  this->declare_parameter<double>("planning_time", 10.0);
  this->declare_parameter<int>("planning_attempts", 10);
  this->declare_parameter<double>("velocity_scaling", 0.5);
  this->declare_parameter<double>("acceleration_scaling", 0.5);

  pick_pos_ = this->get_parameter("pick_position").as_double_array();
  place_pos_ = this->get_parameter("place_position").as_double_array();
  object_name_ = this->get_parameter("object_name").as_string();
  planning_group_ = this->get_parameter("planning_group").as_string();
  end_effector_link_ = this->get_parameter("end_effector_link").as_string();
  planning_time_ = this->get_parameter("planning_time").as_double();
  planning_attempts_ = this->get_parameter("planning_attempts").as_int();
  velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
  acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();

  attach_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/default/create");
  detach_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/default/remove");

  RCLCPP_INFO(this->get_logger(), "PickPlace node created, initializing MoveIt in 5 seconds...");

  init_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
    if (!initialized_) {
      initializeMoveIt();
      initialized_ = true;
    }
  });
}

void PickPlaceNode::initializeMoveIt()
{
  RCLCPP_INFO(this->get_logger(), "Initializing MoveIt...");
  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", ex.what());
    return;
  }
  planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
  move_group_->setEndEffectorLink(end_effector_link_);

  RCLCPP_INFO(this->get_logger(), "MoveIt initialized. Starting pick and place sequence...");
  RCLCPP_INFO(this->get_logger(), "Pick position: [%.3f, %.3f, %.3f]", pick_pos_[0], pick_pos_[1], pick_pos_[2]);
  RCLCPP_INFO(this->get_logger(), "Place position: [%.3f, %.3f, %.3f]", place_pos_[0], place_pos_[1], place_pos_[2]);
  RCLCPP_INFO(this->get_logger(), "End-effector link: %s", end_effector_link_.c_str());

  executePickPlaceSequence();
}

void PickPlaceNode::executePickPlaceSequence()
{
  RCLCPP_INFO(this->get_logger(), "=== Starting Pick and Place Sequence ===");
  RCLCPP_INFO(this->get_logger(), "\n--- Step 1: Moving to pick approach pose ---");
  if (!moveToPickApproach()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to pick approach pose. Aborting sequence.");
    return;
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(this->get_logger(), "\n--- Step 2: Moving to pick pose ---");
  if (!moveToPickPose()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to pick pose. Aborting sequence.");
    return;
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(this->get_logger(), "\n--- Step 3: Attaching object ---");
  if (!attachObject()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to attach object. Aborting sequence.");
    return;
  }
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(this->get_logger(), "\n--- Step 4: Lifting object ---");
  if (!liftObject()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lift object. Aborting sequence.");
    return;
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(this->get_logger(), "\n--- Step 5: Moving to place approach pose ---");
  if (!moveToPlaceApproach()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to place approach pose. Aborting sequence.");
    return;
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(this->get_logger(), "\n--- Step 6: Moving to place pose ---");
  if (!moveToPlacePose()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to place pose. Aborting sequence.");
    return;
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(this->get_logger(), "\n--- Step 7: Detaching object ---");
  if (!detachObject()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to detach object. Continuing anyway...");
  }
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(this->get_logger(), "\n--- Step 8: Moving away ---");
  if (!moveAway()) {
    RCLCPP_WARN(this->get_logger(), "Failed to move away, but sequence mostly completed.");
  }

  RCLCPP_INFO(this->get_logger(), "\n=== Pick and Place Sequence Completed! ===");
}

bool PickPlaceNode::moveToPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  target_pose.orientation = tf2::toMsg(q);

  move_group_->setPoseTarget(target_pose);
  const int max_retries = std::max(1, planning_attempts_);
  bool success = false;

  for (int attempt = 1; attempt <= max_retries; attempt++) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result = move_group_->plan(plan);
    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Planning successful for pose [%.3f, %.3f, %.3f] (attempt %d/%d)", x, y, z, attempt, max_retries);
      moveit::core::MoveItErrorCode execution_result = move_group_->execute(plan);
      if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Execution successful!");
        success = true;
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        break;
      } else {
        RCLCPP_WARN(this->get_logger(), "Execution failed on attempt %d", attempt);
        if (attempt < max_retries) {
          rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed for pose [%.3f, %.3f, %.3f] on attempt %d/%d", x, y, z, attempt, max_retries);
      if (attempt < max_retries) {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to pose [%.3f, %.3f, %.3f] after %d attempts", x, y, z, max_retries);
  }
  return success;
}

bool PickPlaceNode::moveToPickApproach()
{
  return moveToPose(pick_pos_[0], pick_pos_[1], pick_pos_[2] + 0.15, 3.14159, 0.0, 0.0);
}

bool PickPlaceNode::moveToPickPose()
{
  return moveToPose(pick_pos_[0], pick_pos_[1], pick_pos_[2] + 0.02, 3.14159, 0.0, 0.0);
}

bool PickPlaceNode::liftObject()
{
  return moveToPose(pick_pos_[0], pick_pos_[1], pick_pos_[2] + 0.2, 3.14159, 0.0, 0.0);
}

bool PickPlaceNode::moveToPlaceApproach()
{
  return moveToPose(place_pos_[0], place_pos_[1], place_pos_[2] + 0.15, 3.14159, 0.0, 0.0);
}

bool PickPlaceNode::moveToPlacePose()
{
  return moveToPose(place_pos_[0], place_pos_[1], place_pos_[2] + 0.02, 3.14159, 0.0, 0.0);
}

bool PickPlaceNode::moveAway()
{
  return moveToPose(place_pos_[0], place_pos_[1], place_pos_[2] + 0.3, 3.14159, 0.0, 0.0);
}

bool PickPlaceNode::attachObject()
{
  RCLCPP_INFO(this->get_logger(), "Attaching object to vacuum gripper (simulating with fixed joint)");
  RCLCPP_INFO(this->get_logger(), "Waiting for attach service: /world/default/create");
  if (!attach_client_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Attach service /world/default/create not available after 10 seconds");
    RCLCPP_ERROR(this->get_logger(), "Make sure ros_gz_bridge is running and service bridge is configured");
    return false;
  }
  RCLCPP_WARN(this->get_logger(), "NOTE: Actual joint creation is not implemented. Object attachment is simulated.");
  RCLCPP_WARN(this->get_logger(), "To enable real attachment, install gazebo_model_attachment_plugin or use pose-following.");

  // Add object to planning scene and attach it
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.id = object_name_;
  collision_obj.header.frame_id = move_group_->getPlanningFrame();
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.05};
  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = pick_pos_[0];
  obj_pose.position.y = pick_pos_[1];
  obj_pose.position.z = pick_pos_[2];
  obj_pose.orientation.w = 1.0;
  collision_obj.primitives.push_back(box);
  collision_obj.primitive_poses.push_back(obj_pose);
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_obj);
  planning_scene_interface_->applyCollisionObjects(collision_objects);
  move_group_->attachObject(object_name_, move_group_->getEndEffectorLink());
  rclcpp::sleep_for(std::chrono::seconds(1));
  object_attached_ = true;
  RCLCPP_INFO(this->get_logger(), "Object attachment simulated successfully");
  return true;
}

bool PickPlaceNode::detachObject()
{
  RCLCPP_INFO(this->get_logger(), "Simulating detachment of object '%s'", object_name_.c_str());
  RCLCPP_WARN(this->get_logger(), "NOTE: Actual joint removal is not implemented. Object detachment is simulated.");
  try {
    move_group_->detachObject(object_name_);
    planning_scene_interface_->removeCollisionObjects({object_name_});
  } catch (const std::exception &ex) {
    RCLCPP_WARN(this->get_logger(), "Error while detaching object from MoveIt: %s", ex.what());
  }
  rclcpp::sleep_for(std::chrono::seconds(1));
  object_attached_ = false;
  RCLCPP_INFO(this->get_logger(), "Object detachment simulated successfully");
  return true;
}

}  // namespace pick_place

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pick_place::PickPlaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

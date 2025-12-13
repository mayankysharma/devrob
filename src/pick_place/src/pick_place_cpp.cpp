#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <chrono>
#include <thread>
#include <sstream>

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode() : Node("pick_place_cpp")
  {
    // Declare parameters
    this->declare_parameter<std::vector<double>>("pick_position", {0.5, 0.0, 0.85});
    this->declare_parameter<std::vector<double>>("place_position", {-0.5, 0.0, 0.85});
    this->declare_parameter<std::string>("object_name", "pick_object");
    // CRITICAL: Updated planning group name to match standard UR SRDF/MoveIt config
    this->declare_parameter<std::string>("planning_group", "ur5_manipulator"); 
    
    // Get parameters
    pick_pos_ = this->get_parameter("pick_position").as_double_array();
    place_pos_ = this->get_parameter("place_position").as_double_array();
    object_name_ = this->get_parameter("object_name").as_string();
    planning_group_ = this->get_parameter("planning_group").as_string();
    
    // Initialize MoveIt
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    // Set planning parameters
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    
    // Set End-Effector link, this must match the EE link in the SRDF
    move_group_->setEndEffectorLink("tool0"); 
    
    // Create clients for Ignition Gazebo services
    attach_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/default/create");
    detach_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/default/remove");
    
    RCLCPP_INFO(this->get_logger(), "PickPlace node initialized");
    
    // Wait for MoveIt and Gazebo services/topics to be ready
    RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt and Gazebo services...");
    
    // Wait a bit for everything to initialize (MoveIt can take a few seconds)
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // Execute pick and place sequence
    executePickPlace();
  }

private:
  // --- Core Pick and Place Logic ---
  void executePickPlace()
  {
    RCLCPP_INFO(this->get_logger(), "Starting pick and place sequence");
    
    if (!moveToPickApproach()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to pick approach pose. ABORTING.");
      return;
    }
    
    if (!moveToPickPose()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to pick pose. ABORTING.");
      return;
    }
    
    if (!attachObject()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to attach object. ABORTING.");
      return;
    }
    
    if (!liftObject()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to lift object. ABORTING.");
      return;
    }
    
    if (!moveToPlaceApproach()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to place approach pose. ABORTING.");
      return;
    }
    
    if (!moveToPlacePose()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to place pose. ABORTING.");
      return;
    }
    
    if (!detachObject()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to detach object. ABORTING.");
      return;
    }
    
    if (!moveAway()) {
      RCLCPP_WARN(this->get_logger(), "Failed to move away, but operation completed.");
    }
    
    RCLCPP_INFO(this->get_logger(), "Pick and place sequence completed successfully!");
  }
  
  // --- Movement Helper Functions ---
  bool moveToPose(double x, double y, double z, double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    target_pose.orientation = tf2::toMsg(q);
    
    // Use the EE link specified in setEndEffectorLink("tool0")
    move_group_->setPoseTarget(target_pose); 
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group_->execute(plan);
      // Wait for execution to finish
      std::this_thread::sleep_for(std::chrono::seconds(2)); 
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed for target pose [%.2f, %.2f, %.2f]", x, y, z);
      return false;
    }
  }

  bool moveToPickApproach()
  {
    RCLCPP_INFO(this->get_logger(), "Moving to pick approach pose");
    // Standard RPY for wrist facing down
    return moveToPose(
        pick_pos_[0], pick_pos_[1], pick_pos_[2] + 0.15, 3.14159, 0.0, 0.0 
    );
  }
  
  bool moveToPickPose()
  {
    RCLCPP_INFO(this->get_logger(), "Moving to pick pose");
    return moveToPose(
        pick_pos_[0], pick_pos_[1], pick_pos_[2] + 0.02, 3.14159, 0.0, 0.0
    );
  }
  
  bool liftObject()
  {
    RCLCPP_INFO(this->get_logger(), "Lifting object");
    return moveToPose(
        pick_pos_[0], pick_pos_[1], pick_pos_[2] + 0.2, 3.14159, 0.0, 0.0
    );
  }
  
  bool moveToPlaceApproach()
  {
    RCLCPP_INFO(this->get_logger(), "Moving to place approach pose");
    return moveToPose(
        place_pos_[0], place_pos_[1], place_pos_[2] + 0.15, 3.14159, 0.0, 0.0
    );
  }
  
  bool moveToPlacePose()
  {
    RCLCPP_INFO(this->get_logger(), "Moving to place pose");
    return moveToPose(
        place_pos_[0], place_pos_[1], place_pos_[2] + 0.02, 3.14159, 0.0, 0.0
    );
  }
  
  bool moveAway()
  {
    RCLCPP_INFO(this->get_logger(), "Moving away from place position");
    return moveToPose(
        place_pos_[0], place_pos_[1], place_pos_[2] + 0.3, 3.14159, 0.0, 0.0
    );
  }

  // --- Gazebo Attachment Logic (CRITICAL) ---
  bool attachObject()
  {
    RCLCPP_INFO(this->get_logger(), "Attaching object to vacuum gripper (simulating with fixed joint)");
    
    if (!attach_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Attach service not available");
      return false;
    }
    
    // CRITICAL: We create a fixed joint between the 'ur5' model's EE link and the object.
    // Assuming the EE link for the UR5 is 'tool0' (or use your gripper link if defined)
    std::stringstream sdf;
    sdf << "<?xml version='1.0'?>"
        << "<sdf version='1.6'>"
        << "<joint name='attachment_joint_" << object_name_ << "' type='fixed'>"
        << "<parent>ur5::tool0</parent>" // Uses the 'ur5' model name and the 'tool0' link
        << "<child>" << object_name_ << "::link</child>" // Assumes the object model's link is named 'link'
        << "</joint>"
        << "</sdf>";
    
    auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    request->entity_factory.name = "attachment_joint_" + object_name_;
    request->entity_factory.sdf = sdf.str();
    
    auto future = attach_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Object attached successfully");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to attach object");
      return false;
    }
  }
  
  bool detachObject()
  {
    RCLCPP_INFO(this->get_logger(), "Detaching object (deleting fixed joint)");
    
    if (!detach_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Detach service not available");
      return false;
    }
    
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    request->entity.name = "attachment_joint_" + object_name_;
    request->entity.type = ros_gz_interfaces::msg::Entity::JOINT;
    
    auto future = detach_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Object detached successfully");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to detach object");
      return false;
    }
  }
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr attach_client_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr detach_client_;
  
  std::vector<double> pick_pos_;
  std::vector<double> place_pos_;
  std::string object_name_;
  std::string planning_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
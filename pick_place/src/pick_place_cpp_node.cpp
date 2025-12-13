#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <chrono>
#include <vector>
#include <algorithm>

/**
 * @brief A simple pick and place node demonstrating MoveIt! planning and
 * interaction with Ignition Gazebo via ros_gz bridge. This implementation
 * simulates attachment/detachment of an object and executes a conservative
 * pick-and-place trajectory using the robot's end effector.
 */
class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode() : Node("pick_place_cpp"), initialized_(false), object_attached_(false)
  {
    // Declare parameters
    this->declare_parameter<std::vector<double>>("pick_position", {0.6, 0.0, 0.025});
    this->declare_parameter<std::vector<double>>("place_position", {-0.5, 0.0, 0.025});
    this->declare_parameter<std::string>("object_name", "pick_object");
    // Planning group (should match the SRDF group name for the UR robot)
    this->declare_parameter<std::string>("planning_group", "ur5_manipulator"); 
    // End-effector link name (should match SRDF/URDF)
    this->declare_parameter<std::string>("end_effector_link", "tool0");
    // Planning tuning parameters
    this->declare_parameter<double>("planning_time", 10.0);
    this->declare_parameter<int>("planning_attempts", 10);
    this->declare_parameter<double>("velocity_scaling", 0.5);
    this->declare_parameter<double>("acceleration_scaling", 0.5);
    
    // Get parameters
    pick_pos_ = this->get_parameter("pick_position").as_double_array();
    place_pos_ = this->get_parameter("place_position").as_double_array();
    object_name_ = this->get_parameter("object_name").as_string();
    planning_group_ = this->get_parameter("planning_group").as_string();
    end_effector_link_ = this->get_parameter("end_effector_link").as_string();
    planning_time_ = this->get_parameter("planning_time").as_double();
    planning_attempts_ = this->get_parameter("planning_attempts").as_int();
    velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
    acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();
    
    // Create clients for Ignition Gazebo services
    attach_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/default/create");
    detach_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/default/remove");

    // Joint state subscription (used to wait until valid joint states are published by controller_manager)
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [this](sensor_msgs::msg::JointState::UniquePtr msg) {
          last_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>(*msg);
          joint_states_received_ = true;
        }
    );
    
    RCLCPP_INFO(this->get_logger(), "PickPlace node created, initializing MoveIt in 5 seconds...");
    
    // Use a timer to delay MoveIt initialization until after node is fully constructed
    // This avoids the bad_weak_ptr error from calling shared_from_this() in constructor
    init_timer_ = this->create_wall_timer(
        std::chrono::seconds(8),
        [this]() {
          if (!initialized_) {
            initializeMoveIt();
            initialized_ = true;
          }
        });
  }
  
  /**
   * @brief Initialize MoveIt MoveGroup and planning scene with configured parameters
   */
  void initializeMoveIt()
  {
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt...");

    
    // Now safe to use shared_from_this() since node is fully constructed
    // Wait briefly for joint states to be published so MoveIt can use current state as start state
    int wait_seconds = 20;
    for (int i = 0; i < wait_seconds && !joint_states_received_ && rclcpp::ok(); i++) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states message... (timeout %d/%d)", i+1, wait_seconds);
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    if (!joint_states_received_) {
      RCLCPP_WARN(this->get_logger(), "No /joint_states messages received, MoveIt start state may be invalid.");
    }

    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), planning_group_);
    } catch (const std::exception &ex) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", ex.what());
      return;
    }
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    // Set planning parameters
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    
    // Set End-Effector link, this must match the EE link in the SRDF
    move_group_->setEndEffectorLink(end_effector_link_); 

    // Set MoveIt's start state to the current robot state (from joint_states)
    try {
      move_group_->setStartStateToCurrentState();
    } catch (const std::exception &ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to set MoveIt Start State to current state: %s", ex.what());
    }
    
    RCLCPP_INFO(this->get_logger(), "MoveIt initialized. Starting pick and place sequence...");
    RCLCPP_INFO(this->get_logger(), "Pick position: [%.3f, %.3f, %.3f]", 
                pick_pos_[0], pick_pos_[1], pick_pos_[2]);
    RCLCPP_INFO(this->get_logger(), "Place position: [%.3f, %.3f, %.3f]", 
                place_pos_[0], place_pos_[1], place_pos_[2]);
    RCLCPP_INFO(this->get_logger(), "End-effector link: %s", end_effector_link_.c_str());
    RCLCPP_INFO(this->get_logger(), "Planning: time=%.1f attempts=%d vel_scale=%.2f acc_scale=%.2f",
                planning_time_, planning_attempts_, velocity_scaling_, acceleration_scaling_);
    
    // Execute full pick and place sequence
    executePickPlaceSequence();
  }

private:
  // --- Full Pick and Place Sequence ---
  /**
   * @brief Execute a simple pick and place sequence using MoveIt planning
   */
  void executePickPlaceSequence()
  {
    RCLCPP_INFO(this->get_logger(), "=== Starting Pick and Place Sequence ===");
    
    // Step 1: Move to pick approach pose
    RCLCPP_INFO(this->get_logger(), "\n--- Step 1: Moving to pick approach pose ---");
    if (!moveToPickApproach()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to pick approach pose. Aborting sequence.");
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Step 2: Move to pick pose (lower down to object)
    RCLCPP_INFO(this->get_logger(), "\n--- Step 2: Moving to pick pose ---");
    if (!moveToPickPose()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to pick pose. Aborting sequence.");
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Step 3: Attach object (simulate vacuum gripper activation)
    RCLCPP_INFO(this->get_logger(), "\n--- Step 3: Attaching object ---");
    if (!attachObject()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to attach object. Aborting sequence.");
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Step 4: Lift object
    RCLCPP_INFO(this->get_logger(), "\n--- Step 4: Lifting object ---");
    if (!liftObject()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to lift object. Aborting sequence.");
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Step 5: Move to place approach pose
    RCLCPP_INFO(this->get_logger(), "\n--- Step 5: Moving to place approach pose ---");
    if (!moveToPlaceApproach()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to place approach pose. Aborting sequence.");
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Step 6: Move to place pose
    RCLCPP_INFO(this->get_logger(), "\n--- Step 6: Moving to place pose ---");
    if (!moveToPlacePose()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to place pose. Aborting sequence.");
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Step 7: Detach object (simulate vacuum gripper deactivation)
    RCLCPP_INFO(this->get_logger(), "\n--- Step 7: Detaching object ---");
    if (!detachObject()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to detach object. Continuing anyway...");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Step 8: Move away
    RCLCPP_INFO(this->get_logger(), "\n--- Step 8: Moving away ---");
    if (!moveAway()) {
      RCLCPP_WARN(this->get_logger(), "Failed to move away, but sequence mostly completed.");
    }
    
    RCLCPP_INFO(this->get_logger(), "\n=== Pick and Place Sequence Completed! ===");
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
    
    // Use the EE link specified via parameter `end_effector_link`
    move_group_->setPoseTarget(target_pose); 
    
    // Retry logic: try planning up to `planning_attempts_` times (minimum 1)
    const int max_retries = std::max(1, planning_attempts_);
    bool success = false;
    
    for (int attempt = 1; attempt <= max_retries; attempt++) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode planning_result = move_group_->plan(plan);
      
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Planning successful for pose [%.3f, %.3f, %.3f] (attempt %d/%d)", 
                    x, y, z, attempt, max_retries);
        
        // Execute trajectory via MoveIt
        moveit::core::MoveItErrorCode execution_result = move_group_->execute(plan);
        
        if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Execution successful!");
          success = true;
          // Wait for execution to finish
          rclcpp::sleep_for(std::chrono::milliseconds(500));
          break;
        } else {
          RCLCPP_WARN(this->get_logger(), "Execution failed on attempt %d", attempt);
          if (attempt < max_retries) {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          }
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Planning failed for pose [%.3f, %.3f, %.3f] on attempt %d/%d", 
                    x, y, z, attempt, max_retries);
        if (attempt < max_retries) {
          rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
      }
    }
    
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to pose [%.3f, %.3f, %.3f] after %d attempts", 
                   x, y, z, max_retries);
    }
    
    return success;
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

  // --- Gazebo Attachment Logic (NOTE) ---
  bool attachObject()
  {
    RCLCPP_INFO(this->get_logger(), "Attaching object to vacuum gripper (simulating with fixed joint)");
    RCLCPP_INFO(this->get_logger(), "Waiting for attach service: /world/default/create");
    
    // Wait longer for service to become available (might need time for bridge to initialize)
    if (!attach_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Attach service /world/default/create not available after 10 seconds");
      RCLCPP_ERROR(this->get_logger(), "Make sure ros_gz_bridge is running and service bridge is configured");
      return false;
    }
    
    // NOTE: Creating joints between existing models in Ignition Gazebo requires
    // a plugin (like gazebo_model_attachment_plugin) or world-level modifications.
    // SpawnEntity cannot create joints between existing models.
    // For now, we simulate attachment by just logging it.
    RCLCPP_WARN(this->get_logger(), "NOTE: Actual joint creation is not implemented. Object attachment is simulated.");
    RCLCPP_WARN(this->get_logger(), "To enable real attachment, install gazebo_model_attachment_plugin or use pose-following.");
    
    // Also add the object to the MoveIt planning scene and attach it to the EE.
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.id = object_name_;
    collision_obj.header.frame_id = move_group_->getPlanningFrame();
    // Define a small box matching object size; this may be adjusted per URDF
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

    // Add object to planning scene
    // Add object to planning scene (use vector API to support multiple additions)
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_obj);
    planning_scene_interface_->applyCollisionObjects(collision_objects);

    // Attach object to the current end-effector link in MoveIt
    move_group_->attachObject(object_name_, move_group_->getEndEffectorLink());

    // Simulate attachment delay
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Mark as attached for sequence continuation
    object_attached_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Object attachment simulated successfully");
    return true;
  }
  
  bool detachObject()
  {
    RCLCPP_INFO(this->get_logger(), "Simulating detachment of object '%s'", object_name_.c_str());
    RCLCPP_WARN(this->get_logger(), "NOTE: Actual joint removal is not implemented. Object detachment is simulated.");
    
    // Remove attachment in MoveIt and remove collision object from scene
    try {
      move_group_->detachObject(object_name_);
      planning_scene_interface_->removeCollisionObjects({object_name_});
    } catch (const std::exception &ex) {
      RCLCPP_WARN(this->get_logger(), "Error while detaching object from MoveIt: %s", ex.what());
    }
    // Simulate detachment delay
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Mark as detached
    object_attached_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Object detachment simulated successfully");
    return true;
  }
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr attach_client_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr detach_client_;
  
  rclcpp::TimerBase::SharedPtr init_timer_;
  bool initialized_;
  bool object_attached_;  // Track attachment state
  
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
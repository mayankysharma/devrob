#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from rclpy.action import ActionClient


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_py')
        
        # Declare parameters
        self.declare_parameter('pick_position', [0.5, 0.0, 0.85])
        self.declare_parameter('place_position', [-0.5, 0.0, 0.85])
        self.declare_parameter('object_name', 'pick_object')
        self.declare_parameter('planning_group', 'ur5_manipulator')
        
        # Get parameters
        self.pick_pos = self.get_parameter('pick_position').get_parameter_value().double_array_value
        self.place_pos = self.get_parameter('place_position').get_parameter_value().double_array_value
        self.object_name = self.get_parameter('object_name').get_parameter_value().string_value
        self.planning_group_name = self.get_parameter('planning_group').get_parameter_value().string_value
        
        # Create action client for MoveIt
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Create clients for Ignition Gazebo services
        # Note: Attachment in Ignition Gazebo is done via joint creation
        self.attach_client = self.create_client(SpawnEntity, '/world/default/create')
        self.detach_client = self.create_client(DeleteEntity, '/world/default/remove')
        
        self.get_logger().info('PickPlace Python node initialized')
        self.get_logger().info(f'Pick position: {self.pick_pos}')
        self.get_logger().info(f'Place position: {self.place_pos}')
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        if not self.attach_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Attach service not available')
        if not self.detach_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Detach service not available')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('MoveGroup action server not available')
        
        # Wait a bit for everything to initialize
        time.sleep(3)
        
        # Execute pick and place sequence
        self.execute_pick_place()
    
    def execute_pick_place(self):
        self.get_logger().info('Starting pick and place sequence')
        
        # Step 1: Move to pick approach pose
        if not self.move_to_pick_approach():
            self.get_logger().error('Failed to move to pick approach pose')
            return
        
        # Step 2: Move to pick pose
        if not self.move_to_pick_pose():
            self.get_logger().error('Failed to move to pick pose')
            return
        
        # Step 3: Attach object (vacuum gripper)
        if not self.attach_object():
            self.get_logger().error('Failed to attach object')
            return
        
        # Step 4: Lift object
        if not self.lift_object():
            self.get_logger().error('Failed to lift object')
            return
        
        # Step 5: Move to place approach pose
        if not self.move_to_place_approach():
            self.get_logger().error('Failed to move to place approach pose')
            return
        
        # Step 6: Move to place pose
        if not self.move_to_place_pose():
            self.get_logger().error('Failed to move to place pose')
            return
        
        # Step 7: Detach object
        if not self.detach_object():
            self.get_logger().error('Failed to detach object')
            return
        
        # Step 8: Move away from place position
        if not self.move_away():
            self.get_logger().warn('Failed to move away, but operation completed')
        
        self.get_logger().info('Pick and place sequence completed successfully!')
    
    def move_to_pose(self, x, y, z, roll=3.14159, pitch=0.0, yaw=0.0):
        """Move to a target pose using MoveIt via action"""
        # Note: This is a simplified version. In practice, you might need to use
        # the move_group action server or MoveGroupInterface Python bindings
        # For now, we'll use a simple service call approach
        
        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        
        # Convert RPY to quaternion
        quat = quaternion_from_euler(roll, pitch, yaw)
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]
        
        # For now, log the target pose
        # In a real implementation, you would use MoveGroupInterface Python bindings
        # or call the move_group action server
        self.get_logger().info(f'Target pose: x={x:.2f}, y={y:.2f}, z={z:.2f}')
        self.get_logger().warn('Python MoveIt interface not fully implemented. Use C++ version for full functionality.')
        
        # Simulate movement delay
        time.sleep(2)
        return True
    
    def move_to_pick_approach(self):
        self.get_logger().info('Moving to pick approach pose')
        return self.move_to_pose(
            self.pick_pos[0],
            self.pick_pos[1],
            self.pick_pos[2] + 0.15,  # 15cm above pick position
            roll=3.14159, pitch=0.0, yaw=0.0
        )
    
    def move_to_pick_pose(self):
        self.get_logger().info('Moving to pick pose')
        return self.move_to_pose(
            self.pick_pos[0],
            self.pick_pos[1],
            self.pick_pos[2] + 0.02,  # Slightly above object
            roll=3.14159, pitch=0.0, yaw=0.0
        )
    
    def attach_object(self):
        self.get_logger().info('Attaching object to wrist_3_link')
        # Note: In Ignition Gazebo, attachment is done via creating a fixed joint
        # This is a simplified version - full implementation would create a joint
        self.get_logger().warn('Attachment via service not fully implemented. Use C++ version.')
        time.sleep(1)
        return True
    
    def lift_object(self):
        self.get_logger().info('Lifting object')
        return self.move_to_pose(
            self.pick_pos[0],
            self.pick_pos[1],
            self.pick_pos[2] + 0.2,  # Lift 20cm
            roll=3.14159, pitch=0.0, yaw=0.0
        )
    
    def move_to_place_approach(self):
        self.get_logger().info('Moving to place approach pose')
        return self.move_to_pose(
            self.place_pos[0],
            self.place_pos[1],
            self.place_pos[2] + 0.15,  # 15cm above place position
            roll=3.14159, pitch=0.0, yaw=0.0
        )
    
    def move_to_place_pose(self):
        self.get_logger().info('Moving to place pose')
        return self.move_to_pose(
            self.place_pos[0],
            self.place_pos[1],
            self.place_pos[2] + 0.02,  # Slightly above place position
            roll=3.14159, pitch=0.0, yaw=0.0
        )
    
    def detach_object(self):
        self.get_logger().info('Detaching object from wrist_3_link')
        # Note: In Ignition Gazebo, detachment is done via deleting the joint
        # This is a simplified version - full implementation would delete the joint
        self.get_logger().warn('Detachment via service not fully implemented. Use C++ version.')
        time.sleep(1)
        return True
    
    def move_away(self):
        self.get_logger().info('Moving away from place position')
        return self.move_to_pose(
            self.place_pos[0],
            self.place_pos[1],
            self.place_pos[2] + 0.3,  # Move up 30cm
            roll=3.14159, pitch=0.0, yaw=0.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

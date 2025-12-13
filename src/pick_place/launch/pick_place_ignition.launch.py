import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # --- 1. Get Package Directories ---
    pick_place_pkg_path = get_package_share_directory('pick_place')
    ur_description_pkg_path = get_package_share_directory('ur_description')
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    
    # --- 2. Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    arg_x = DeclareLaunchArgument('x', default_value='0.0')
    arg_y = DeclareLaunchArgument('y', default_value='0.0')
    arg_z = DeclareLaunchArgument('z', default_value='0.0')  # On the ground
    
    # Pick and place position arguments
    arg_pick_pos = DeclareLaunchArgument('pick_position', default_value='[0.6, 0.0, 0.025]',
                                         description='Pick position [x, y, z]')
    arg_place_pos = DeclareLaunchArgument('place_position', default_value='[-0.5, 0.0, 0.025]',
                                          description='Place position [x, y, z]') 

    # --- 3. Construct Paths ---
    
    # Use local URDF which has correct plugin names for gz_ros2_control
    urdf_file_path = os.path.join(pick_place_pkg_path, "urdf", "ur5_vacuum.urdf.xacro")

    srdf_file_path = os.path.join(pick_place_pkg_path, "config", "ur5_manipulator.srdf")
    world_file_path = os.path.join(pick_place_pkg_path, "worlds", "pick_place_world.sdf")
    controller_config_file = os.path.join(pick_place_pkg_path, "config", "ur5_controllers.yaml")

    # --- 4. MoveIt Configuration ---
    moveit_controllers_file = os.path.join(pick_place_pkg_path, "config", "moveit_controllers.yaml")
    moveit_config = (
        MoveItConfigsBuilder("ur5_manipulator", package_name="pick_place")
        .robot_description(file_path=urdf_file_path)      
        .robot_description_semantic(file_path=srdf_file_path)
        .trajectory_execution(file_path=moveit_controllers_file)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # --- 5. Nodes ---
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description, 
            {'use_sim_time': use_sim_time}
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ur5_manipulator',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            controller_config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            # Explicitly set controller manager namespace for Gazebo plugin
            {'moveit_controller_manager_ns': '/controller_manager'}
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # Bridge for clock topic
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Bridge for Gazebo services (SpawnEntity and DeleteEntity)
    # These services allow creating/deleting entities (like joints) in the simulation
    # Format: service@ROS2_srv_type@Ign_req_type@Ign_rep_type
    service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean',
            '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity@gz.msgs.Entity@gz.msgs.Boolean',
        ],
        output='screen'
    )
    
    pick_place_app = Node(
        package="pick_place",
        executable='pick_place_cpp_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            # Pick and place positions (can be overridden via --ros-args -p pick_position:="[x, y, z]")
            {'pick_position': [0.6, 0.0, 0.025]},  # Default: matches cube position in world (0.6, 0.0, 0.025)
            {'place_position': [-0.5, 0.0, 0.025]}  # Default: matches place marker in world (-0.5, 0.0, 0.025)
        ]
    )

    # --- 6. Event Handlers ---

    # Note: For Ignition/Gazebo, the controller manager is provided by the Gazebo plugin
    # We don't need a standalone ros2_control_node. The plugin handles it.
    # Wait for spawn to complete, then start controllers via the plugin's manager
    delay_broadcaster = RegisterEventHandler(
        OnProcessStart(target_action=spawn_entity, on_start=[
            TimerAction(period=2.0, actions=[joint_state_broadcaster])
        ])
    )

    delay_jtc = RegisterEventHandler(
        OnProcessStart(target_action=joint_state_broadcaster, on_start=[
            TimerAction(period=1.0, actions=[joint_trajectory_controller])
        ])
    )

    delay_moveit = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_trajectory_controller,
            on_start=[move_group_node, TimerAction(period=5.0, actions=[pick_place_app])]
        )
    )

    return LaunchDescription([
        arg_use_sim_time, arg_x, arg_y, arg_z, arg_pick_pos, arg_place_pos,
        gz_sim, clock_bridge, service_bridge, robot_state_publisher, spawn_entity,
        delay_broadcaster, delay_jtc, delay_moveit,
    ])
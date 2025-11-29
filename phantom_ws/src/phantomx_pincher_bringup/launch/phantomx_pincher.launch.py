from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paths to important files
    urdf_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_description"),
        "urdf",
        "phantomx_pincher.urdf.xacro",
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_description"),
        "rviz",
        "urdf_config.rviz",
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_bringup"),
        "config",
        "controllers_position.yaml",
    ])

    move_group_launch_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_moveit_config"),
        "launch",
        "move_group.launch.py",
    ])

    # Robot State Publisher (loads URDF via xacro)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_path])
        }],
        output="screen",
    )

    # ros2_control node with POSITION controllers
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="screen",
    )

    # Spawners – names must match controllers_position.yaml
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_trajectory_controller"],
        output="screen",
    )

    # Include MoveIt move_group launch file
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([move_group_launch_path])
    )

    # RViz2 with predefined config
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_launch,
        rviz_node,
    ])

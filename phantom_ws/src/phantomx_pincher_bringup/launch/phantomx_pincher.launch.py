from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -------------------------------------------------------------------------
    #  Choose SIM vs REAL explicitly
    # -------------------------------------------------------------------------
    use_real_robot = LaunchConfiguration("use_real_robot")

    use_real_robot_arg = DeclareLaunchArgument(
        "use_real_robot",
        default_value="false",
        description=(
            "If 'true', connect MoveIt to the real PhantomX via "
            "pincher_control/follow_joint_trajectory. "
            "If 'false', use ros2_control simulation."
        ),
    )

    # -------------------------------------------------------------------------
    #  Paths (shared)
    # -------------------------------------------------------------------------
    urdf_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_description"),
        "urdf",
        "phantomx_pincher.urdf.xacro",
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_moveit_config"),
        "config",
        "controllers_position.yaml",
    ])

    move_group_launch_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_moveit_config"),
        "launch",
        "move_group.launch.py",
    ])

    # Robot description string (xacro → URDF)
    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str,
    )

    # -------------------------------------------------------------------------
    #  Common nodes (both SIM and REAL)
    # -------------------------------------------------------------------------

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
        }],
    )

    # Commander (MoveGroupInterface wrapper) – always run
    commander_node = Node(
        package="phantomx_pincher_commander_cpp",
        executable="commander",
        name="commander",
        output="screen",
    )

    # -------------------------------------------------------------------------
    #  SIMULATION stack (ros2_control fake hardware)
    # -------------------------------------------------------------------------

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        condition=UnlessCondition(use_real_robot),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        condition=UnlessCondition(use_real_robot),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
        ],
        condition=UnlessCondition(use_real_robot),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_trajectory_controller",
            "--controller-manager", "/controller_manager",
        ],
        condition=UnlessCondition(use_real_robot),
    )

    move_group_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([move_group_launch_path]),
        launch_arguments={
            # Empty string → bool('') == False inside move_group.launch.py
            # so it will NOT start its own ros2_control_node
            "ros2_control": "",
            # We already manage controllers in this launch file
            "manage_controllers": "false",
            # We want MoveIt RViz
            "enable_rviz": "true",
        }.items(),
        condition=UnlessCondition(use_real_robot),
    )

    # -------------------------------------------------------------------------
    #  REAL HARDWARE stack (pincher_control follow_joint_trajectory)
    # -------------------------------------------------------------------------

    follow_joint_trajectory_node = Node(
        package="pincher_control",
        executable="follow_joint_trajectory",
        name="pincher_follow_joint_trajectory",
        output="screen",
        # Optional: override defaults if needed
        # parameters=[{
        #     "port": "/dev/ttyUSB0",
        #     "baudrate": 1000000,
        #     "joint_prefix": "phantomx_pincher_",
        #     "moving_speed": 200,
        #     "torque_limit": 400,
        #     "gripper_id": 5,
        # }],
        condition=IfCondition(use_real_robot),
    )

    move_group_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([move_group_launch_path]),
        launch_arguments={
            # Do NOT let MoveIt start ros2_control
            "ros2_control": "false",
            # Matches the mode you used manually with move_group.launch.py
            "ros2_control_plugin": "real",
            # pincher_control provides the action servers, so MoveIt won't spawn controllers
            "manage_controllers": "false",
            "enable_rviz": "true",
        }.items(),
        condition=IfCondition(use_real_robot),
    )

    # -------------------------------------------------------------------------
    #  LaunchDescription
    # -------------------------------------------------------------------------
    return LaunchDescription([
        use_real_robot_arg,

        # Common
        robot_state_publisher_node,
        commander_node,

        # SIM-only
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_sim,

        # REAL-only
        follow_joint_trajectory_node,
        move_group_real,
    ])

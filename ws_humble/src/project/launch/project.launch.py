import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    OrSubstitution,
)
from launch_param_builder import ParameterBuilder

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.3",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur3e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic

def generate_launch_description():
    # arm_type = LaunchConfiguration("arm_type")
    # arm_type_arg = DeclareLaunchArgument("arm_type", default="ur3e")
    # arm_type = "panda"
    arm_type = "ur3e"
    moveit_panda_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs())
    if arm_type == "ur3e":
        frame = "base_link"
        robot_description = get_robot_description()
        robot_description_semantic = get_robot_description_semantic()
        # Get parameters for the Servo node
        servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
        servo_params = {"moveit_servo": servo_yaml}
        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"]
        )
        ros2_controllers_path = (
            get_package_share_directory("ur_moveit_config") + "/config/controllers.yaml"
        )
    elif arm_type == "panda":
        frame = "panda_link0"
        robot_description = moveit_panda_config.robot_description
        robot_description_semantic = moveit_panda_config.robot_description_semantic
        servo_params = {
            "moveit_servo": ParameterBuilder("moveit_servo")
                .yaml("config/panda_simulated_config.yaml")
                .to_dict() }
        rviz_config_file = (
            get_package_share_directory("moveit_servo") + "/config/demo_rviz_config.rviz"
        )
        ros2_controllers_path = (
            get_package_share_directory("moveit_resources_panda_moveit_config")
            + "/config/ros2_controllers.yaml"
        )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        # Maybe it isn't worth having these multithreaded.
        executable="component_container_mt",
        composable_node_descriptions=([
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    robot_description,
                    robot_description_semantic,
                    moveit_panda_config.robot_description_kinematics,
                    moveit_panda_config.joint_limits,
                ],
            ),
        ] if arm_type == "panda" else []) + [
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),
            ComposableNode(
                package="project",
                plugin="project::Controller",
                name="controller",
                parameters=[{"frame": frame}],
            ),
            ComposableNode(
                package="project",
                plugin="project::PhyhoxStreamer",
                name="phyphox_streamer",
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": frame, "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        ([
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
        ] if arm_type == "panda" else []) + [container]
    )

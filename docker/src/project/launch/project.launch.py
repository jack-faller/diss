import xacro
from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    OrSubstitution,
)
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    return ParameterBuilder(package_name).yaml(file_path).to_dict()

class RobotDescription:
    def __init__(self, description, semantic, kinematics):
        self.description = description
        self.semantic = semantic
        self.kinematics = kinematics

def generate_launch_description():
    frame = "link0"

    moveit_config = (
        MoveItConfigsBuilder("kinova_gen3_lite")
        .robot_description(file_path="config/gen3_lite.urdf.xacro")
        .to_moveit_configs())
    servo_params = { "moveit_servo": ParameterBuilder("kortex_bringup", "config/servo.yaml") }
    # ros2_controllers_path = (
    #     get_package_share_directory("kortex_moveit_config") + "kinova_gen3_lite_moveit_config/config/ros2_controllers.yaml"
    # )

    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="screen",
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager-timeout",
    #         "300",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # panda_arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["kortex_arm_controller", "-c", "/controller_manager"],
    # )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        # Maybe it isn't worth having these multithreaded.
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
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
        kinova_nodes + [
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # panda_arm_controller_spawner,
            container,
        ]
    )

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
    moveit_config = MoveItConfigsBuilder("kinova_gen3_lite").to_moveit_configs()
    servo_yaml = load_yaml("kortex_bringup", "config/servo.yaml")
    # This is set to "manipulator" by default for some reason.
    # servo_yaml['move_group_name'] = 'gripper'
    servo_params = { "moveit_servo": servo_yaml }
    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="container",
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
                    # {'use_intra_process_comms': True},
                    servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
            # ComposableNode(
            #     package="robot_state_publisher",
            #     plugin="robot_state_publisher::RobotStatePublisher",
            #     name="robot_state_publisher",
            #     parameters=[moveit_config.robot_description],
            # ),

            ComposableNode(
                package="project",
                plugin="project::Controller",
                name="controller",
                parameters=[{"frame": servo_yaml['planning_frame']}],
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
                parameters=[{"child_frame_id": servo_yaml['planning_frame'], "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # panda_arm_controller_spawner,

            container,
        #     Node(package="moveit_servo",
        # executable="servo_node_main",
        # name="servo_node",
        # parameters=[
        #     servo_params,
        #     moveit_config.robot_description,
        #     moveit_config.robot_description_semantic,
        #     moveit_config.robot_description_kinematics,
        #     moveit_config.joint_limits,
        # ],
        # output="screen",)
        # ]
    )

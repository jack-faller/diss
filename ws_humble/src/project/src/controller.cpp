#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <Eigen/Core>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;
using std::placeholders::_1;

const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string ACCEL_TOPIC = "/controller/accel_cmds";
const std::string BASE_FRAME_ID = "base_link";

typedef Eigen::Vector3d Vec3;

namespace project
{
  class Controller : public rclcpp::Node
{
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
  rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr twist_sub;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client;

  std::string frame_to_publish;

  std::thread collision_pubthread;
  rclcpp::Time t {0};
  Vec3 v = {};
  Vec3 p = {};
public:
  Controller(const rclcpp::NodeOptions& options)
  : Node("controller", options), frame_to_publish(BASE_FRAME_ID)
  {
    RCLCPP_INFO(get_logger(), "STARTED %d", 4);
    twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    collision_pub =
      this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

    // Wait for servo to start.
    servo_start_client = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client->wait_for_service(1s);
    servo_start_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    auto cb = std::bind(&Controller::publishCB, this, _1);
    twist_sub = this->create_subscription<geometry_msgs::msg::Accel>(ACCEL_TOPIC, rclcpp::SystemDefaultsQoS(), cb);

    // Load the collision scene asynchronously
    collision_pubthread = std::thread([this]() {
      rclcpp::sleep_for(3s);
      // Create collision object, in the way of servoing
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = BASE_FRAME_ID;
      collision_object.id = "box";

      shape_msgs::msg::SolidPrimitive table_1;
      table_1.type = table_1.BOX;
      table_1.dimensions = { 0.4, 0.6, 0.03 };

      geometry_msgs::msg::Pose table_1_pose;
      table_1_pose.position.x = 0.6;
      table_1_pose.position.y = 0.0;
      table_1_pose.position.z = 0.4;

      shape_msgs::msg::SolidPrimitive table_2;
      table_2.type = table_2.BOX;
      table_2.dimensions = { 0.6, 0.4, 0.03 };

      geometry_msgs::msg::Pose table_2_pose;
      table_2_pose.position.x = 0.0;
      table_2_pose.position.y = 0.5;
      table_2_pose.position.z = 0.25;

      collision_object.primitives.push_back(table_1);
      collision_object.primitive_poses.push_back(table_1_pose);
      // collision_object.primitives.push_back(table_2);
      // collision_object.primitive_poses.push_back(table_2_pose);
      collision_object.operation = collision_object.ADD;

      moveit_msgs::msg::PlanningSceneWorld psw;
      psw.collision_objects.push_back(collision_object);

      auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
      ps->world = psw;
      ps->is_diff = true;
      collision_pub->publish(std::move(ps));
    });
  }

  ~Controller() override
  {
    if (collision_pubthread.joinable())
      collision_pubthread.join();
  }

  void publishCB(geometry_msgs::msg::Accel::SharedPtr msg)
  {
    auto now = this->now();
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.frame_id = frame_to_publish;
    twist_msg->header.stamp = now;

    auto then = t;
    t = now;
    if (then.nanoseconds() == 0)
      return;
    double dt = (now - then).nanoseconds() / 10e9;

    // TODO: Quaternion maths here for angular momentum.
    Vec3 a(msg->linear.x, msg->linear.y, msg->linear.z);
    Vec3 dv = a * dt;
    Vec3 dp = dv * dt / 2 + v * dt;
    Vec3 v_out = v + dv / 2;
    twist_msg->twist.linear.x = v_out.x();
    twist_msg->twist.linear.y = v_out.y();
    twist_msg->twist.linear.z = v_out.z();
    twist_pub->publish(std::move(twist_msg));
    v += dv;
    p += dp;
  }
};

}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(project::Controller)

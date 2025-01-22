#include <geometry_msgs/msg/accel_stamped.hpp>
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
#include <chrono>
#include <vector>
#include <thread>
#include <fstream>
#include <sstream>
using namespace std::chrono_literals;

const std::string ACCEL_TOPIC = "/controller/accel_cmds";

namespace project
{
  class PhyhoxStreamer : public rclcpp::Node
{
  std::atomic_bool terminate = false;
  rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub;
  struct Entry { double t, x, y, z; };
  std::vector<Entry> file;
  std::thread executor;
  rclcpp::TimerBase::SharedPtr timer;
public:
  PhyhoxStreamer(const rclcpp::NodeOptions& options)
  : Node("phyhox_streamer", options)
  {
    accel_pub = this->create_publisher<geometry_msgs::msg::Accel>(ACCEL_TOPIC, rclcpp::SystemDefaultsQoS());

    std::ifstream in("/proj/Acceleration without g 2024-11-27 06-16-18/Raw Data.csv");
    if (in.fail())
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
      std::istringstream iss(line);
      Entry e;
      double abs;
      char c1, c2, c3, c4;
      in >> e.t >> c1 >> e.x >> c2 >> e.y >> c3 >> e.z >> c4 >> abs;
      file.push_back(e);
    }

    executor = std::thread([this]() {
      rclcpp::sleep_for(5s);
      double t = 0;
      for (size_t i = 0; i < file.size() && !terminate; t = file[i].t, ++i) {
        auto delta = file[i].t - t;
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta)));
        auto accel_msg = std::make_unique<geometry_msgs::msg::Accel>();
        accel_msg->linear.x = file[i].x;
        accel_msg->linear.y = file[i].y;
        accel_msg->linear.z = file[i].z;
        accel_pub->publish(std::move(accel_msg));
      }
    });
  }

  ~PhyhoxStreamer() override
  {
    terminate = true;
    if (executor.joinable())
      executor.join();
  }
};

}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(project::PhyhoxStreamer)

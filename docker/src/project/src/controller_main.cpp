#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

namespace project
{
  class Controller : public rclcpp::Node { public: Controller(const rclcpp::NodeOptions& options); };
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //! For the options, we pass a default-constructed rvalue.
  auto node = std::make_shared<project::Controller>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

#include <openrst_control/openrst_control.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace openrst_nu;

bool kill_this_process = false;

void SigIntHandler(int signal)
{
  (void)signal;
  kill_this_process = true;
  rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, SigIntHandler);

  auto orst_node = std::make_shared<OpenRSTControl>("openrst_control", &kill_this_process);

  // Use a multithreaded executor to handle callbacks while running the control loop
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(orst_node);

  std::thread t_executor([&executor]() {
      executor.spin();
  });

  orst_node->ControlLoop();

  if (t_executor.joinable()) {
      executor.cancel();
      t_executor.join();
  }

  return 0;
}
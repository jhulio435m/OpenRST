#include <rclcpp/rclcpp.hpp>
#include <usb3104/usb3104.h>
#include <csignal>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("usb3104_stream_node");

  int cycle_freq;
  node->declare_parameter("usb3104_stream_freq", 100);
  node->get_parameter("usb3104_stream_freq", cycle_freq);

  RCLCPP_INFO(node->get_logger(), "USB3104_stream_node: freq = %d Hz", cycle_freq);
  rclcpp::Rate loop_rate(cycle_freq);

  USB3104 daq_ao(node, "01F7E9D2");
  if (daq_ao.InitAO() != 0) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize AO");
      return -1;
  }

  while (rclcpp::ok())
  {
    daq_ao.PublishStateAO();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  daq_ao.Quit();
  rclcpp::shutdown();
  return 0;
}
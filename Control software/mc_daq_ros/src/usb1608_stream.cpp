#include <rclcpp/rclcpp.hpp>
#include <usb1608/usb1608.h>
#include <csignal>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("usb1608_stream_node");

  int cycle_freq;
  node->declare_parameter("usb1608_stream_freq", 100);
  node->get_parameter("usb1608_stream_freq", cycle_freq);

  RCLCPP_INFO(node->get_logger(), "USB1608_stream_node: freq = %d Hz", cycle_freq);
  rclcpp::Rate loop_rate(cycle_freq);

  // Note: ID should probably be a parameter too
  USB1608 daq_ai(node, "01F92A95");

  if (daq_ai.InitAI() != 0) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize AI");
      return -1;
  }
  daq_ai.InitDIO(4, 4);
  daq_ai.StartScanAI();

  while (rclcpp::ok())
  {
    daq_ai.UpdateScanStateAI();
    daq_ai.PublishStateAI();
    daq_ai.UpdateStateDI();
    daq_ai.PublishStateDI();
    
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  daq_ai.StopScanAI();
  daq_ai.Quit();
  rclcpp::shutdown();

  return 0;
}
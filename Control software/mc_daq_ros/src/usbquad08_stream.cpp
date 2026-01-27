#include <rclcpp/rclcpp.hpp>
#include <usbquad08/usbquad08.h>
#include <csignal>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("usbquad08_stream_node");

  int cycle_freq;
  node->declare_parameter("usbquad08_stream_freq", 100);
  node->get_parameter("usbquad08_stream_freq", cycle_freq);

  RCLCPP_INFO(node->get_logger(), "USBQUAD08_stream_node: freq = %d Hz", cycle_freq);
  rclcpp::Rate loop_rate(cycle_freq);

  USBQUAD08 daq_enc(node, "1001633");
  if (daq_enc.InitENC() != 0) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize ENC");
      return -1;
  }
  daq_enc.StartScanENC();

  while (rclcpp::ok())
  {
    daq_enc.UpdateScanStateENC();
    daq_enc.PublishStateENC();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  daq_enc.StopScanENC();
  daq_enc.Quit();
  rclcpp::shutdown();
  return 0;
}
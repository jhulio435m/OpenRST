#ifndef USB3104_H
#define USB3104_H

// C
#include <stdio.h>
#include <stdlib.h>

// C++
#include <vector>
#include <string>
#include <memory>

// External
#include "uldaq.h"
#include <ul_lib/utility.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class USB3104
{
  const int kNumberOfAoChannels = 8;

public:
  USB3104(std::shared_ptr<rclcpp::Node> nh, std::string daq_id);
  ~USB3104();

  int InitAO();
  int UpdateStateAO();
  int UpdateStateChannelAO(int channel);

  int PublishStateAO();
  void Quit();

  void PrintError(UlError err_);

  std::vector<double> GetAoState();

  bool IsEnabledAO();
  // Accessors
  int get_ao_channels();
  // Mutators
  void set_ao_cmd(std::vector<double> ao_cmd);
  // Callbacks
  void UpdateAOValueCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

private:
  // ROS
  std::shared_ptr<rclcpp::Node> nh_;

  // ROS Topics
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ao_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ao_cmd_;
  std_msgs::msg::Float64MultiArray pub_ao_state_msg_;

  // Device
  int descriptorIndex_;
  DaqDeviceDescriptor devDescriptor_;
  DaqDeviceInterface interfaceType_;
  DaqDeviceHandle daqDeviceHandle_;
  unsigned int numDevs_;
  UlError err_;

  std::string daq_id_;
  bool daq_ready_;
  bool daq_connected_;

  // Analog Output vars
  Range range_;
  AOutFlag flags_;
  int numberOfChannels_;
  std::vector<double> ao_state_;
  std::vector<double> ao_cmd_;
  bool ao_enabled_;
};

#endif // USB3104_H

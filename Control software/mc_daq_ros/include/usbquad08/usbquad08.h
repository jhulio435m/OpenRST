#ifndef USBQUAD08_H
#define USBQUAD08_H

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
#include <std_msgs/msg/int32_multi_array.hpp>

// Internal
#include <mc_daq_ros/srv/daq_cmd.hpp>

class USBQUAD08
{
  const int kNumberOfEncChannels = 8;

public:
  USBQUAD08(std::shared_ptr<rclcpp::Node> nh, std::string daq_id);
  ~USBQUAD08();

  void SrvDaqCommandCb(const std::shared_ptr<mc_daq_ros::srv::DaqCmd::Request> req,
                       std::shared_ptr<mc_daq_ros::srv::DaqCmd::Response> res);

  int InitENC();
  int SetZero(int enc_id);
  int SetAllZero();

  int UpdateStateENC();
  int PublishStateENC();
  void Quit();

  int StartScanENC();
  int StopScanENC();
  int UpdateScanStateENC();

  std::vector<int> GetEncState();

  void PrintError(UlError err_);

  bool IsEnabledENC();
  int get_enc_channels();

  void set_offset(int value);

private:
  // ROS
  std::shared_ptr<rclcpp::Node> nh_;

  // ROS Topics
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_enc_state_;
  std_msgs::msg::Int32MultiArray pub_enc_state_msg_;

  rclcpp::Service<mc_daq_ros::srv::DaqCmd>::SharedPtr srv_server_daq_cmd;

  // Device
  int descriptorIndex_;
  DaqDeviceDescriptor devDescriptor_;
  DaqDeviceInterface interfaceType_;
  DaqDeviceHandle daqDeviceHandle_;
  unsigned int numDevs_;
  UlError err_;

  // encoder settings
  CounterMeasurementType type_;
  CounterMeasurementMode mode_;
  CounterEdgeDetection edgeDetection_;
  CounterTickSize tickSize_;
  CounterDebounceMode debounceMode_;
  CounterDebounceTime debounceTime_;
  CConfigScanFlag configFlags_;
  CInScanFlag flags_;

  std::string daq_id_;
  bool daq_ready_;
  bool daq_connected_;

  // Encoder vars
  int numberOfEncoders_;
  std::vector<int> enc_state_;
  bool enc_enabled_;
  unsigned long long *enc_buffer_ = NULL;
  int samplesPerCounter_;
  double rate_;
  ScanOption scanOptions_;

  int offset_;
};

#endif // USBQUAD08_H
#ifndef USB1608_H
#define USB1608_H

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
#include <std_msgs/msg/int32_multi_array.hpp>

// Internal
#include <math_utils/filters.h>

class USB1608
{
  const int kNumberOfAiChannels = 16;
  const int kNumberOfDioPorts = 1;
  const int kNumberOfDioIO = 8;
  const int kNumberOfCntChannels = 2;
  const int kNumberOfTimers = 1;
  const int kMaxDevCount = 10;
  const int kMaxStrlength = 64;

public:
  USB1608(std::shared_ptr<rclcpp::Node> nh, std::string daq_id);
  ~USB1608();

  int InitDIO(int n_din, int n_dout);
  int InitDIO();
  int InitAI();

  int UpdateStateDI(int bit_number);
  int UpdateStateDI();
  int PublishStateDI();

  int UpdateStateDO(int bit_number, unsigned int bit_value);
  int UpdateStateDO();
  int UpdateStateAI();
  int PublishStateAI();
  void Quit();

  int StartScanAI();
  int UpdateScanStateAI();
  int StopScanAI();

  std::vector<double> GetScanStateChannelsAI(int start_ch, int n_channels);

  void PrintError(UlError err_);

  bool IsEnabledDIO() { return dio_enabled_; }
  bool IsEnabledAI() { return ai_enabled_; }

  std::vector<double> get_ai_state() { return ai_state_; }
  std::vector<int> get_di_state() { return di_state_; }

  int get_ai_channels();
  int get_di_channels();
  int get_do_channels();

  // Callbacks
  void UpdateDOValueCb(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

private:
  // ROS
  std::shared_ptr<rclcpp::Node> nh_;

  // ROS Topics
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ai_state_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ai_state_filtered;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_di_state_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_do_cmd_;
  
  std_msgs::msg::Float64MultiArray pub_ai_state_msg_;
  std_msgs::msg::Float64MultiArray pub_ai_state_filtered_msg_;
  std_msgs::msg::Int32MultiArray pub_di_state_msg_;

  filters::IIR *filter_iir_ai;
  VectorXd ai_state_filtered_;

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

  // Analog Input vars
  AiInputMode inputMode_;
  Range ai_range_;
  int numberOfChannels_;
  int samplesPerChannel_;
  std::vector<double> ai_state_;
  bool ai_enabled_;
  double *ai_buffer_ = NULL;
  double ai_rate_;
  ScanOption ai_scanOptions_;
  AInScanFlag flags_ai_scan_;
  AInFlag flags_;

  // Digital Port vars
  DigitalPortType portType_;
  DigitalPortIoType portIoType_;

  int bitsPerPort_;
  int n_dinput_;
  int n_doutput_;
  std::vector<int> di_state_;
  std::vector<int> do_cmd_;
  bool dio_enabled_;
};

#endif // USB1608_H

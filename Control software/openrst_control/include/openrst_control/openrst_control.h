#ifndef OPENRST_CONTROL_H
#define OPENRST_CONTROL_H

// C
#include <pthread.h>
#include <pwd.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

// C++
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

// External
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
// Note: controller_manager in ROS 2 is different, we might not use it directly inside the node
// unless we are doing something very specific. 

// Internal
#include <openrst_control/srv/openrst_request.hpp>
#include <openrst_control/openrst_status_code.h>
#include <rt_utils/rt_clock.h>
#include <mc_daq_ros/srv/daq_cmd.hpp>

#include <Eigen/Dense>

using namespace Eigen;

namespace openrst_nu
{
  const double kDeg2Rad = (M_PI) / 180.0;
  const double kRad2Deg = 180.0 / (M_PI);

  class OpenRSTControl : public rclcpp::Node
  {
    enum
    {
      CALIB_STOP = -1,
      CALIB_READY,
      CALIB_M0_HIGH,
      CALIB_M0_LOW,
      SET_M0_ZERO,
      CALIB_M1_HIGH,
      CALIB_M2_HIGH,
      CALIB_M1_LOW,
      SET_M1_HIGH,
      CALIB_M2_LOW,
      SET_M2_ZERO,
      SET_M1_ZERO,
      CALIB_COMPLETED
    };

    struct ArmJointLimits
    {
      std::string name;
      bool has_position_limits = {false};
      double min_pos = {0.0};
      double max_pos = {0.0};
      bool has_velocity_limits = {false};
      double max_vel = {0.0};
      bool has_effort_limits = {false};
      double max_eff = {0.0};
    };

  public:
    OpenRSTControl(const std::string & node_name, bool *kill_this_node);
    ~OpenRSTControl();

    // Callbacks
    void SrvOpenRSTCommandCb(const std::shared_ptr<openrst_control::srv::OpenrstRequest::Request> request,
                             std::shared_ptr<openrst_control::srv::OpenrstRequest::Response> response);

    void SubUpdateSimJointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void SubUpdateJointCommandCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void SubUpdateAiStateCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void SubUpdateDiStateCb(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void SubUpdateAoStateCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void SubUpdateEncStateCb(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    int ControlLoop();
    void PublishRobotState();

    bool connect();
    bool disconnect();
    bool calibrate();
    bool posControlOn();
    bool torqueControlOn();
    bool controlOff();

    bool IsModeChanged();
    int get_mode();

    void SafeSleepSeconds(double seconds);
    void BlockSleepSeconds(double useconds);
    void SendCmdRobot();
    void SendCmdSim();
    void Read();
    void Write();

  private:
    // DAQ channels variables
    int kAiIndexCurrMotor0 = 0;
    int kAiIndexCurrMotor1 = 1;
    int kAiIndexCurrMotor2 = 2;
    int kAiIndexSensor0 = 8;
    int kAiIndexSensor1 = 9;
    int kAiIndexSensor2 = 10;
    int kAoIndexMotor0 = 0;
    int kAoIndexMotor1 = 1;
    int kAoIndexMotor2 = 2;
    int kEncIndexMotor0 = 0;
    int kEncIndexMotor1 = 1;
    int kEncIndexMotor2 = 2;
    
    double kSensorTresh0 = 0.12;
    double kSensorTresh1 = 0.12;
    double kSensorTresh2 = 0.12;
    
    int kRatioMotorPitch = 3640;
    int kRatioMotorFinger = 4096;
    double kCalibMotorCurr = 0.1;
    double kMotorCurrToControlSignal = 50.0;

    double kMotorKp0 = 0.003;
    double kMotorKp1 = 0.0045;
    double kMotorKp2 = 0.006;

    // Robot variables
    int openrst_id_;
    int n_dof_;

    // Services
    rclcpp::Service<openrst_control::srv::OpenrstRequest>::SharedPtr srv_server_openrst_request_;
    rclcpp::Client<mc_daq_ros::srv::DaqCmd>::SharedPtr srv_client_daq_cmd_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_sim_joint_state_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_joint_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ai_state_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_di_state_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ao_state_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_enc_state_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_state_;
    std_msgs::msg::Int32MultiArray pub_state_msg_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_sim_joint_cmd_;
    sensor_msgs::msg::JointState pub_sim_joint_cmd_msg_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ao_cmd_;
    std_msgs::msg::Float64MultiArray pub_ao_cmd_msg_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_do_cmd_;
    std_msgs::msg::Int32MultiArray pub_do_cmd_msg_;

    // Joint variables
    VectorXd des_joint_pos_;
    VectorXd des_joint_eff_;
    VectorXd act_joint_pos_;
    VectorXd sim_joint_pos_;
    VectorXd daq_joint_pos_;

    VectorXd enc_offset_;
    VectorXd enc_joint_range_;
    VectorXd enc_range_;
    VectorXd enc_coeff_;

    // DAQ variables
    std::vector<double> ai_state_;
    std::vector<int> di_state_;
    std::vector<double> ao_state_;
    std::vector<int> enc_state_;
    std::vector<double> ao_cmd_;
    std::vector<int> do_cmd_;

    // RT
    realtime_utils::RTClock rt_clock_;
    int cycle_t_us_;

    // State
    int state_;
    bool *kill_this_node_;

    // Mutex
    std::mutex m_mtxAct;

    std::vector<ArmJointLimits> joint_limits_;
    std::vector<std::string> joint_names_;

    bool use_sim_;
    bool mode_changed_;
    int mode_;

    rclcpp::Time now_timestamp_;
    rclcpp::Time prev_timestamp_;
    rclcpp::Duration period_ = rclcpp::Duration::from_nanoseconds(0);

    int ai_channels_;
    int ao_channels_;
    int di_channels_;
    int do_channels_;
    int enc_channels_;
  };
} // namespace openrst_nu

#endif
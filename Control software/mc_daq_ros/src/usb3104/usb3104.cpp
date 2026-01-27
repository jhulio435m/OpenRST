#include <usb3104/usb3104.h>
#include <unistd.h>

#define MAX_DEV_COUNT 10
#define MAX_STR_LENGTH 64

USB3104::USB3104(std::shared_ptr<rclcpp::Node> nh, std::string daq_id)
    : nh_(nh), daq_id_(daq_id)
{
  daq_ready_ = false;
  daq_connected_ = false;

  descriptorIndex_ = 0;
  interfaceType_ = USB_IFC;
  daqDeviceHandle_ = 0;
  numDevs_ = MAX_DEV_COUNT;

  numberOfChannels_ = kNumberOfAoChannels;

  ao_state_.resize(numberOfChannels_);
  flags_ = AOUT_FF_DEFAULT;

  int i = 0;
  err_ = ERR_NO_ERROR;
  DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];

  err_ = ulGetDaqDeviceInventory(interfaceType_, devDescriptors, &numDevs_);

  if (err_ != ERR_NO_ERROR)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error getting DAQ boards information: %d", err_);
    return;
  }

  if (numDevs_ == 0)
  {
    RCLCPP_ERROR(nh_->get_logger(), "No DAQ device is detected");
    return;
  }

  bool found = false;
  for (i = 0; i < (int)numDevs_; i++)
  {
    if (devDescriptors[i].uniqueId == daq_id_)
    {
      devDescriptor_ = devDescriptors[i];
      found = true;
      break;
    }
  }

  if (!found)
  {
    RCLCPP_ERROR(nh_->get_logger(), "DAQ Board with ID %s not found", daq_id_.c_str());
    return;
  }

  RCLCPP_INFO(nh_->get_logger(), "Creating DAQ Handle for ID %s", daq_id_.c_str());
  daqDeviceHandle_ = ulCreateDaqDevice(devDescriptor_);
  if (daqDeviceHandle_ == 0)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Unable to create a handle for %s DAQ device", daq_id_.c_str());
    return;
  }

  daq_ready_ = true;

  // Publishers
  pub_ao_state_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>("/usb3104/ao/state", 1);

  // Subscribers
  sub_ao_cmd_ = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/usb3104/ao/cmd", 1000, std::bind(&USB3104::UpdateAOValueCb, this, std::placeholders::_1));

  pub_ao_state_msg_.data.resize(numberOfChannels_);
  ao_cmd_.resize(numberOfChannels_, 0.0);
}

USB3104::~USB3104() {}

int USB3104::InitAO()
{
  int hasAO = 0;
  char rangeStr[64];
  double min = 0.0;
  double max = 0.0;

  if (!daq_ready_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "DAQ board is not ready");
    return -1;
  }

  err_ = getDevInfoHasAo(daqDeviceHandle_, &hasAO);
  if (!hasAO)
  {
    RCLCPP_ERROR(nh_->get_logger(), "The specified DAQ device does not support analog output");
    return -1;
  }

  RCLCPP_WARN(nh_->get_logger(), "Connecting to device %s - please wait ...", devDescriptor_.devString);

  int attempts = 10;
  for (int trial = 0; trial < attempts; trial++)
  {
    err_ = ulConnectDaqDevice(daqDeviceHandle_);
    if (err_ == ERR_NO_ERROR)
    {
      RCLCPP_INFO(nh_->get_logger(), "Connected to device %s", devDescriptor_.devString);
      break;
    }
    else
    {
      RCLCPP_WARN(nh_->get_logger(), "Connection to device %s failed, attempt %d of %d", devDescriptor_.devString, trial + 1, attempts);
      if (trial == attempts - 1)
      {
        RCLCPP_ERROR(nh_->get_logger(), "Connection to device %s failed, exiting", devDescriptor_.devString);
        return -1;
      }
      sleep(1);
    }
  }

  daq_connected_ = true;

  err_ = getAoInfoFirstSupportedRange(daqDeviceHandle_, &range_, rangeStr);
  ConvertRangeToMinMax(range_, &min, &max);

  RCLCPP_INFO(nh_->get_logger(), "ANALOG OUTPUT INFO");
  RCLCPP_INFO(nh_->get_logger(), "\tRange: %s", rangeStr);

  ao_enabled_ = true;
  return 0;
}

void USB3104::UpdateAOValueCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  for (size_t ch = 0; ch < msg->data.size() && ch < (size_t)numberOfChannels_; ch++)
  {
    if (msg->data[ch] >= -10.0 && msg->data[ch] <= 10.0)
    {
      ao_cmd_[ch] = msg->data[ch];
      UpdateStateChannelAO(ch);
    }
  }
}

int USB3104::UpdateStateAO()
{
  if (!ao_enabled_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "AO Module is not enabled");
    return -1;
  }

  if ((int)ao_cmd_.size() != numberOfChannels_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "# AO Channels doesn't match command size");
    return -1;
  }

  for (int chan = 0; chan < 6; chan++) // Logic matches original: only first 6?
  {
    err_ = ulAOut(daqDeviceHandle_, chan, range_, flags_, ao_cmd_.at(chan));
    if (err_ == ERR_NO_ERROR)
    {
      ao_state_.at(chan) = ao_cmd_.at(chan);
    }
    else
    {
      RCLCPP_ERROR(nh_->get_logger(), "Error setting AO command");
      return -1;
    }
  }
  ao_state_ = ao_cmd_;
  return 0;
}

int USB3104::UpdateStateChannelAO(int channel)
{
  if (!ao_enabled_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "AO Module is not enabled");
    return -1;
  }

  if ((int)ao_cmd_.size() != numberOfChannels_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "# AO Channels doesn't match command size");
    return -1;
  }

  err_ = ulAOut(daqDeviceHandle_, channel, range_, flags_, ao_cmd_.at(channel));
  if (err_ == ERR_NO_ERROR)
  {
    ao_state_.at(channel) = ao_cmd_.at(channel);
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error setting AO command for channel %d", channel);
    return -1;
  }
  return 0;
}

int USB3104::PublishStateAO()
{
  pub_ao_state_msg_.data = ao_state_;
  pub_ao_state_->publish(pub_ao_state_msg_);
  return 0;
}

bool USB3104::IsEnabledAO() { return ao_enabled_; }

void USB3104::Quit()
{
  ulDisconnectDaqDevice(daqDeviceHandle_);
  if (daqDeviceHandle_)
    ulReleaseDaqDevice(daqDeviceHandle_);
}

void USB3104::PrintError(UlError err_)
{
  if (err_ != ERR_NO_ERROR)
  {
    char errMsg[ERR_MSG_LEN];
    ulGetErrMsg(err_, errMsg);
    RCLCPP_ERROR(nh_->get_logger(), "Error Code: %d", err_);
    RCLCPP_ERROR(nh_->get_logger(), "Error Message: %s", errMsg);
  }
}

void USB3104::set_ao_cmd(std::vector<double> ao_cmd) { ao_cmd_ = ao_cmd; }
int USB3104::get_ao_channels() { return numberOfChannels_; }
std::vector<double> USB3104::GetAoState() { return ao_state_; }
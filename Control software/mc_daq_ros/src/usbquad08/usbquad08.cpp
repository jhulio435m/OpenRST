#include <usbquad08/usbquad08.h>
#include <unistd.h>

#define MAX_DEV_COUNT 100
#define MAX_SCAN_OPTIONS_LENGTH 256
#define MAX_ENCODER_COUNTERS 16

USBQUAD08::USBQUAD08(std::shared_ptr<rclcpp::Node> nh, std::string daq_id)
    : nh_(nh), daq_id_(daq_id)
{
  daq_ready_ = false;
  daq_connected_ = false;

  descriptorIndex_ = 0;
  interfaceType_ = USB_IFC;
  daqDeviceHandle_ = 0;
  numDevs_ = MAX_DEV_COUNT;

  type_ = CMT_ENCODER;
  mode_ = (CounterMeasurementMode)(CMM_ENCODER_X4 | CMM_ENCODER_CLEAR_ON_Z);
  edgeDetection_ = CED_RISING_EDGE;
  tickSize_ = CTS_TICK_20ns;
  debounceMode_ = CDM_NONE;
  debounceTime_ = CDT_DEBOUNCE_0ns;
  configFlags_ = CF_DEFAULT;
  flags_ = CINSCAN_FF_DEFAULT;

  samplesPerCounter_ = 10;
  rate_ = 10000;
  scanOptions_ = (ScanOption)(SO_DEFAULTIO | SO_CONTINUOUS);

  enc_state_.resize(kNumberOfEncChannels);
  offset_ = 0;
  numberOfEncoders_ = kNumberOfEncChannels;

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
  pub_enc_state_ = nh_->create_publisher<std_msgs::msg::Int32MultiArray>("/usbquad08/enc/state", 1);
  pub_enc_state_msg_.data.resize(kNumberOfEncChannels);

  // Services
  srv_server_daq_cmd = nh_->create_service<mc_daq_ros::srv::DaqCmd>(
      "daq_command", std::bind(&USBQUAD08::SrvDaqCommandCb, this, std::placeholders::_1, std::placeholders::_2));
}

USBQUAD08::~USBQUAD08() {}

void USBQUAD08::SrvDaqCommandCb(const std::shared_ptr<mc_daq_ros::srv::DaqCmd::Request> req,
                                 std::shared_ptr<mc_daq_ros::srv::DaqCmd::Response> res)
{
  RCLCPP_INFO(nh_->get_logger(), "DAQ Command Called with request: %s", req->message.c_str());
  if (req->message == "reset_enc")
  {
    SetZero(req->port);
    res->result = true;
  }
  else if (req->message == "reset_all_enc")
  {
    SetAllZero();
    res->result = true;
  }
  else
  {
      res->result = false;
  }
}

int USBQUAD08::InitENC()
{
  int hasCI = 0;
  int hasPacer = 0;
  int encoderCounters[MAX_ENCODER_COUNTERS];

  if (!daq_ready_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "DAQ board is not ready");
    return -1;
  }

  err_ = getDevInfoHasCtr(daqDeviceHandle_, &hasCI);
  if (!hasCI)
  {
    RCLCPP_ERROR(nh_->get_logger(), "The specified DAQ device does not support counter input");
    return -1;
  }

  err_ = getCtrInfoHasPacer(daqDeviceHandle_, &hasPacer);
  if (!hasPacer)
  {
    RCLCPP_ERROR(nh_->get_logger(), "The specified DAQ device does not support hardware paced counter input");
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

  err_ = getCtrInfoSupportedEncoderCounters(daqDeviceHandle_, encoderCounters, &numberOfEncoders_);
  if (numberOfEncoders_ == 0)
  {
    RCLCPP_ERROR(nh_->get_logger(), "The specified DAQ device does not support encoder channels");
  }

  for (int i = 0; i < numberOfEncoders_; i++)
  {
    err_ = ulCConfigScan(daqDeviceHandle_, i, type_, mode_, edgeDetection_,
                         tickSize_, debounceMode_, debounceTime_, configFlags_);
  }

  RCLCPP_INFO(nh_->get_logger(), "ENCODER INFO");
  RCLCPP_INFO(nh_->get_logger(), "\t# Encoders: %d", numberOfEncoders_);

  SetAllZero();
  RCLCPP_INFO(nh_->get_logger(), "Encoders initialized as Zero");
  enc_enabled_ = true;
  return 0;
}

int USBQUAD08::StartScanENC()
{
  if (!enc_enabled_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "ENC Module is not enabled");
    return -1;
  }

  enc_buffer_ = (unsigned long long *)malloc(numberOfEncoders_ * samplesPerCounter_ * sizeof(unsigned long long));
  if (enc_buffer_ == NULL)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Out of memory, unable to create scan buffer");
    return -1;
  }

  err_ = ulCInScan(daqDeviceHandle_, 0, numberOfEncoders_ - 1, samplesPerCounter_,
                   &rate_, scanOptions_, flags_, enc_buffer_);

  if (err_ == ERR_NO_ERROR)
    return 0;

  return -1;
}

int USBQUAD08::StopScanENC()
{
  ScanStatus status;
  TransferStatus transferStatus;

  err_ = ulCInScanStatus(daqDeviceHandle_, &status, &transferStatus);
  if (status == SS_RUNNING && err_ == ERR_NO_ERROR)
  {
    err_ = ulCInScanStop(daqDeviceHandle_);
  }

  if (enc_buffer_) {
      free(enc_buffer_);
      enc_buffer_ = NULL;
  }
  return 0;
}

int USBQUAD08::SetZero(int enc_id)
{
  ulCClear(daqDeviceHandle_, enc_id);
  return 0;
}

int USBQUAD08::SetAllZero()
{
  for (int chan = 0; chan < numberOfEncoders_; chan++)
  {
    ulCClear(daqDeviceHandle_, chan);
  }
  return 0;
}

void USBQUAD08::set_offset(int value)
{
  offset_ = value;
}

int USBQUAD08::UpdateStateENC()
{
  unsigned long long data = 0;
  if (!enc_enabled_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "ENC Module is not enabled");
    return -1;
  }

  for (int chan = 0; chan < numberOfEncoders_; chan++)
  {
    err_ = ulCIn(daqDeviceHandle_, chan, &data);
    if (err_ == ERR_NO_ERROR)
      enc_state_.at(chan) = (int)data;
  }
  return 0;
}

int USBQUAD08::UpdateScanStateENC()
{
  ScanStatus status;
  TransferStatus transferStatus;

  err_ = ulCInScanStatus(daqDeviceHandle_, &status, &transferStatus);
  int index = transferStatus.currentIndex;

  for (int ch = 0; ch < numberOfEncoders_; ch++)
  {
    if (err_ == ERR_NO_ERROR)
    {
      enc_state_.at(ch) = (int)enc_buffer_[index + ch];
      if (enc_state_[ch] > 32768)
        enc_state_[ch] -= 65536;
    }
  }
  return 0;
}

bool USBQUAD08::IsEnabledENC() { return enc_enabled_; }

int USBQUAD08::PublishStateENC()
{
  pub_enc_state_msg_.data = enc_state_;
  pub_enc_state_->publish(pub_enc_state_msg_);
  return 0;
}

void USBQUAD08::Quit()
{
  ulDisconnectDaqDevice(daqDeviceHandle_);
  if (daqDeviceHandle_)
    ulReleaseDaqDevice(daqDeviceHandle_);
}

void USBQUAD08::PrintError(UlError err_)
{
  if (err_ != ERR_NO_ERROR)
  {
    char errMsg[ERR_MSG_LEN];
    ulGetErrMsg(err_, errMsg);
    RCLCPP_ERROR(nh_->get_logger(), "Error Code: %d", err_);
    RCLCPP_ERROR(nh_->get_logger(), "Error Message: %s", errMsg);
  }
}

int USBQUAD08::get_enc_channels() { return numberOfEncoders_; }
std::vector<int> USBQUAD08::GetEncState() { return enc_state_; }
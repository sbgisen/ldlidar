#include <ldlidar_node.h>

#include <lifecycle_msgs/msg/state.hpp>


LD06::LD06() : rclcpp_lifecycle::LifecycleNode("ld06_node"), updater_(this), consecutive_read_failures_(0)
{
  std::string topic_name = this->declare_parameter("topic_name", "scan");
  this->declare_parameter("serial_port", ""); //TODO: Figure out what's the real port name
  this->declare_parameter("lidar_frame", "laser");
  this->declare_parameter("range_threshold", 0.005);
  this->declare_parameter<double>("diagnostic_tolerance", 0.1);
  this->declare_parameter<int>("max_consecutive_read_failures", 10);
  auto reconnection_duration = this->declare_parameter<double>("reconnection_duration", 1.0);

  auto pub = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);

  updater_.setHardwareID("ld_laser");
  auto tolerance = this->get_parameter("diagnostic_tolerance").as_double();
  frequency_ = 10.0;

  diagnosed_publisher_ = std::make_shared<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::LaserScan>>(
    pub, updater_, diagnostic_updater::FrequencyStatusParam(&frequency_, &frequency_, tolerance, 10),
    diagnostic_updater::TimeStampStatusParam());
  auto_recovery_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(reconnection_duration), [this] { autoRecoveryTrigger(); });
}

auto LD06::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) -> LD06::CallbackReturn
{
  std::string port_name = this->get_parameter("serial_port").as_string();
  std::string lidar_frame = this->get_parameter("lidar_frame").as_string();
  float range_threshold = this->get_parameter("range_threshold").as_double();
  lidar_ = new LiPkg;

  lidar_->SetLidarFrame(lidar_frame);
  lidar_->SetRangeThreshold(range_threshold);
  if (port_name.empty()) {
    RCLCPP_INFO(this->get_logger(), "Autodetecting serial port");
    std::vector<std::pair<std::string, std::string>> device_list;
    cmd_port_.GetCmdDevices(device_list);
    auto found = std::find_if(device_list.begin(), device_list.end(), [](std::pair<std::string, std::string> n) {
      return strstr(n.second.c_str(), "CP2102");
    });

    if (found != device_list.end()) {
      RCLCPP_INFO(this->get_logger(), "%s %s", found->first.c_str(), found->second.c_str());
      port_name = found->first;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Can't find LiDAR LD06");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Using port %s", port_name.c_str());

  cmd_port_.SetReadCallback([this](const char * byte, size_t len) {
    if (lidar_->Parse((uint8_t *)byte, len)) {
      lidar_->AssemblePacket();
    }
  });

  if (cmd_port_.Open(port_name)) {
    RCLCPP_INFO(this->get_logger(), "LiDAR_LD06 started successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Can't open the serial port");
    return LD06::CallbackReturn::FAILURE;
  }

  return LD06::CallbackReturn::SUCCESS;
}

auto LD06::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) -> LD06::CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Activating LD06 node, starting data reading.");
  consecutive_read_failures_ = 0;

  loop_timer_ = this->create_wall_timer(100ms, std::bind(&LD06::publishLoop, this));
  return LD06::CallbackReturn::SUCCESS;
}

auto LD06::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) -> LD06::CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Deactivating LD06 node, stopping data reading.");
  if (loop_timer_) {
    loop_timer_->cancel();
    loop_timer_.reset();
  }
  return LD06::CallbackReturn::SUCCESS;
}

auto LD06::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) -> LD06::CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up LD06 node, closing port and resetting state.");
  if (cmd_port_.IsOpened()) {
    cmd_port_.Close();
  }
  consecutive_read_failures_ = 0;
  return LD06::CallbackReturn::SUCCESS;
}

auto LD06::on_error(const rclcpp_lifecycle::State & /*previous_state*/) -> LD06::CallbackReturn
{
  RCLCPP_ERROR(this->get_logger(), "An error occurred in the LD06 node.");
  if (loop_timer_) {
    loop_timer_->cancel();
    loop_timer_.reset();
  }
  if (cmd_port_.IsOpened()) {
    cmd_port_.Close();
  }
  consecutive_read_failures_ = 0;
  return LD06::CallbackReturn::SUCCESS;
}

void LD06::publishLoop()
{
  if (lidar_->IsFrameReady())
  {
    diagnosed_publisher_->publish(lidar_->GetLaserScan());
    consecutive_read_failures_ = 0;
    lidar_->ResetFrameReady();
  }else{
    consecutive_read_failures_++;
    if (consecutive_read_failures_ >= this->get_parameter("max_consecutive_read_failures").as_int() &&
        get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
  }
}

void LD06::autoRecoveryTrigger()
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    auto port = this->get_parameter("serial_port").as_string();
    if (!port.empty() && access(port.c_str(), F_OK) == 0) {
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }
  } else if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (consecutive_read_failures_ > 0) {
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    } else if (!loop_timer_) {
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }
  }
}
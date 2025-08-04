#ifndef LD06_H
#define LD06_H

#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tofbf.h"
#include <string>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

using namespace std::chrono_literals;

class LD06 : public rclcpp_lifecycle::LifecycleNode
{
  public:
    LD06();
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_activate(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_deactivate(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_cleanup(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_error(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;

  private:
    CmdInterfaceLinux cmd_port_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::TimerBase::SharedPtr auto_recovery_timer_;
    LiPkg * lidar_;
    double frequency_;
    diagnostic_updater::Updater updater_;
    std::shared_ptr<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::LaserScan>> diagnosed_publisher_;
    int consecutive_read_failures_;
    void publishLoop();
    void autoRecoveryTrigger();
};

#endif
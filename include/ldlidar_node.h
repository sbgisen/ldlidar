#ifndef LD06_H
#define LD06_H

#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tofbf.h"
#include <string>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

using namespace std::chrono_literals;

class LD06 : public rclcpp::Node
{
  public:
    LD06();

  private:
    CmdInterfaceLinux cmd_port_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    LiPkg * lidar_;
    double frequency_;
    diagnostic_updater::Updater updater_;
    std::shared_ptr<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::LaserScan>> diagnosed_publisher_;
    void publishLoop();
};

#endif
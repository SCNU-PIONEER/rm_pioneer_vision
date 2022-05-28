// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "rm_serial_driver/rm_serial_driver.hpp"

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  present_color_(1),
  present_mode_(0),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // Create Publisher
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  auto_aim_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  energy_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "energy_predictor");

  // get remote parameters
  RCLCPP_INFO(this->get_logger(), "Try to get remote param");
  while ((!auto_aim_param_client_->service_is_ready()) &&
         (!energy_param_client_->service_is_ready())) {
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  requestforChangeColor(present_color_);
  requestforChangeMode(present_mode_);

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/processor/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  // Latency Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          sensor_msgs::msg::JointState joint_state;
          joint_state.header.stamp = this->now();
          joint_state.name.push_back("pitch_joint");
          joint_state.name.push_back("yaw_joint");
          joint_state.position.push_back(packet.pitch);
          joint_state.position.push_back(packet.yaw);
          joint_state_pub_->publish(joint_state);

          if (packet.robot_color != present_color_) {
            requestforChangeColor(packet.robot_color);
          }
          if (packet.task_mode != present_mode_) {
            requestforChangeMode(packet.task_mode);
          }
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN(get_logger(), "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  try {
    SendPacket packet;
    packet.target_found = msg->target_found;
    packet.target_color = present_color_ == 0;
    packet.task_mode = 0;
    packet.x = msg->position.x;
    packet.y = msg->position.y;
    packet.z = msg->position.z;
    packet.vx = msg->velocity.x;
    packet.vy = msg->velocity.y;
    packet.vz = msg->velocity.z;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::requestforChangeColor(uint8_t color)
{
  bool set_auto_aim_success = false;
  bool set_energy_success = false;
  // Parameter Client
  if (auto_aim_param_client_->service_is_ready()) {
    auto_aim_param_client_->set_parameters(
      {rclcpp::Parameter("detect_color", color == 0 ? 1 : 0)},
      [&](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        future.wait();
        auto results = future.get();
        set_auto_aim_success = results[0].successful;
      });
    if (set_auto_aim_success) {
      present_color_ = color;
      RCLCPP_INFO(get_logger(), "Successfully set color: %d", color);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set color");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "auto aim parameter server is not ready");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (energy_param_client_->service_is_ready()) {
    energy_param_client_->set_parameters(
      {rclcpp::Parameter("detect_color", color == 0 ? 1 : 0)},
      [&](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        future.wait();
        auto results = future.get();
        set_energy_success = results[0].successful;
      });
    if (set_energy_success) {
      present_color_ = color;
      RCLCPP_INFO(get_logger(), "Successfully set color: %d", color);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set color");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "auto aim parameter server is not ready");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}

void RMSerialDriver::requestforChangeMode(uint8_t mode)
{
  bool set_auto_aim_success = false;
  bool set_energy_success = false;
  if (auto_aim_param_client_->service_is_ready()) {
    auto_aim_param_client_->set_parameters(
      {rclcpp::Parameter("active", mode ? false : true)},
      [&](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        future.wait();
        auto results = future.get();
        set_auto_aim_success = results[0].successful;
      });
    if (set_auto_aim_success) {
      present_mode_ = mode;
      RCLCPP_INFO(get_logger(), "Successfully set auto aim mode: %d", mode);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set auto aim active");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "auto aim parameter server is not ready");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (energy_param_client_->service_is_ready()) {
    energy_param_client_->set_parameters(
      {rclcpp::Parameter("task_mode", mode)},
      [&](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        future.wait();
        auto results = future.get();
        set_energy_success = results[0].successful;
      });
    if (set_energy_success) {
      present_mode_ = mode;
      RCLCPP_INFO(get_logger(), "Successfully set energy mode: %d", mode);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set energy mode");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "energy parameter server is not ready");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}
}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)

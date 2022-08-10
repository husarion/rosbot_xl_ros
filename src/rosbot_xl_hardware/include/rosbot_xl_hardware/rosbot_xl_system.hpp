#ifndef ROSBOT_XL_HARDWARE__ROSBOT_XL_SYSTEM_HPP_
#define ROSBOT_XL_HARDWARE__ROSBOT_XL_SYSTEM_HPP_

#include "rosbot_xl_hardware/visibility_control.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "sensor_msgs/msg/joint_state.hpp"

namespace rosbot_xl_hardware
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

using JointState = sensor_msgs::msg::JointState;

class RosbotXLSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RosbotXLSystem)

  ROSBOT_XL_HARDWARE_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  ROSBOT_XL_HARDWARE_PUBLIC
  std::vector<CommandInterface> export_command_interfaces() override;

  ROSBOT_XL_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

  ROSBOT_XL_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  ROSBOT_XL_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  ROSBOT_XL_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  ROSBOT_XL_HARDWARE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  ROSBOT_XL_HARDWARE_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  ROSBOT_XL_HARDWARE_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  ROSBOT_XL_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  ROSBOT_XL_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  realtime_tools::RealtimeBox<std::shared_ptr<JointState>> received_motor_state_msg_ptr_{ nullptr };

  std::shared_ptr<rclcpp::Publisher<JointState>> motor_command_publisher_ = nullptr;

  std::shared_ptr<realtime_tools::RealtimePublisher<JointState>> realtime_motor_command_publisher_ = nullptr;

  rclcpp::Subscription<JointState>::SharedPtr motor_state_subscriber_ = nullptr;

  std::map<std::string, double> vel_commands_;
  std::map<std::string, double> pos_state_;
  std::map<std::string, double> vel_state_;

  bool subscriber_is_active_ = false;

  std::shared_ptr<rclcpp::Node> node_;

  void motor_state_cb(const std::shared_ptr<JointState> msg);
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::unique_ptr<std::thread> executor_thread_;
};

}  // namespace rosbot_xl_hardware

#endif  // ROSBOT_XL_HARDWARE__ROSBOT_XL_SYSTEM_HPP_
#ifndef ROSBOT_XL_HARDWARE__ROSBOT_XL_HARDWARE_HPP_
#define ROSBOT_XL_HARDWARE__ROSBOT_XL_HARDWARE_HPP_

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

#include "std_msgs/msg/float64_multi_array.hpp"

namespace rosbot_xl_hardware
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

using Float64MultiArray = std_msgs::msg::Float64MultiArray;

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
  return_type read() override;

  ROSBOT_XL_HARDWARE_PUBLIC
  return_type write() override;

protected:
  realtime_tools::RealtimeBox<std::shared_ptr<Float64MultiArray>> received_motor_state_msg_ptr_{ nullptr };

  std::shared_ptr<rclcpp::Publisher<Float64MultiArray>> motor_command_publisher_ = nullptr;

  std::shared_ptr<realtime_tools::RealtimePublisher<Float64MultiArray>> realtime_motor_command_publisher_ = nullptr;

  rclcpp::Subscription<Float64MultiArray>::SharedPtr motor_state_subscriber_ = nullptr;

  std::vector<double> vel_commands_;
  std::vector<double> pos_state_;
  std::vector<double> vel_state_;

  bool subscriber_is_active_ = false;

  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace rosbot_xl_hardware

#endif  // ROSBOT_XL_HARDWARE__ROSBOT_XL_HARDWARE_HPP_
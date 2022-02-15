#include "rosbot_xl_hardware/rosbot_xl_system.hpp"

#include <string>
#include <vector>

#include "rclcpp/logging.hpp"


#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"

namespace rosbot_xl_hardware {

CallbackReturn RosbotXLSystem::on_init(const hardware_interface::HardwareInfo & hardware_info) {

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  // catch (const std::exception & e) {
  //   fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  //   RCLCPP_FATAL(
  //     rclcpp::get_logger("RosbotXLSystem"),
  //     "Exception thrown during init stage with message: %s \n", e.what());
  //   return CallbackReturn::ERROR;
  // }


  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   // DiffBotSystem has exactly two states and one command interface on each joint
  //   if (joint.command_interfaces.size() != 1)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::g  is_halted = false;et_logger("RosbotXLSystem"),
  //       "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
  //       joint.command_interfaces.size());
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("RosbotXLSystem"),
  //       "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
  //       joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces.size() != 2)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("RosbotXLSystem"),
  //       "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
  //       joint.state_interfaces.size());
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("RosbotXLSystem"),
  //       "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
  //       joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("RosbotXLSystem"),
  //       "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
  //       joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
  //     return CallbackReturn::ERROR;
  //   }
  // }


  node_ = std::make_shared<rclcpp::Node>(
    "hardware_node", rclcpp::NodeOptions()
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true));

  pos_state_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  vel_state_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  vel_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}




std::vector<StateInterface> RosbotXLSystem::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_state_[i]));
  }

  return state_interfaces;
}



std::vector<CommandInterface> RosbotXLSystem::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]));
  }

  return command_interfaces;
}






CallbackReturn RosbotXLSystem::on_configure(const rclcpp_lifecycle::State & ) {
  RCLCPP_INFO(rclcpp::get_logger("RosbotXLSystem"), "Starting");

  motor_command_publisher_ =
    node_->create_publisher<Float64MultiArray>("~/motor/command", rclcpp::SystemDefaultsQoS());
  realtime_motor_command_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(motor_command_publisher_); 

  motor_state_subscriber_ = node_->create_subscription<Float64MultiArray>(
    "~/motors/state", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<Float64MultiArray> msg) -> void {
      if (!subscriber_is_active_) {
        RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      received_motor_state_msg_ptr_.set(std::move(msg));
  });

  return CallbackReturn::SUCCESS;
}


CallbackReturn RosbotXLSystem::on_activate(const rclcpp_lifecycle::State & ) {
  // method where hardware “power” is enabled.
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}


CallbackReturn RosbotXLSystem::on_deactivate(const rclcpp_lifecycle::State & ) {
  // method, which does the opposite of on_activate.
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}


CallbackReturn RosbotXLSystem::on_cleanup(const rclcpp_lifecycle::State & ) {
  // method, which does the opposite of on_configure.
  return CallbackReturn::SUCCESS;
}


CallbackReturn RosbotXLSystem::on_shutdown(const rclcpp_lifecycle::State & ) {
  // method where hardware is shutdown gracefully.
  return CallbackReturn::SUCCESS;
}


CallbackReturn RosbotXLSystem::on_error(const rclcpp_lifecycle::State & ) {
  // method where different errors from all states are handled.
  return CallbackReturn::SUCCESS;
}



return_type RosbotXLSystem::read() {
  std::shared_ptr<Float64MultiArray> motor_state;
  received_motor_state_msg_ptr_.get(motor_state);

  for (auto i = 0u; i < motor_state->layout.dim[0].size/2; i++) {
    pos_state_[i] = motor_state->data[i];
    vel_state_[i] = motor_state->data[i + motor_state->layout.dim[0].size/2];
  }

  return return_type::OK;
}


return_type RosbotXLSystem::write() {
  if (realtime_motor_command_publisher_->trylock()) {
    auto & motor_command = realtime_motor_command_publisher_->msg_;
    // motor_command.layout.dim = new std_msgs::msg::MultiArrayDimension;
    // motor_command.layout.dim[0].label = "control_signal";
    // motor_command.layout.dim[0].size = vel_commands_.size();
    // motor_command.layout.dim[0].size = vel_commands_.size();
    // motor_command.layout.data_offset = 0u;

    for (auto i = 0u; i < vel_commands_.size(); i++) {
      motor_command.data[i] = vel_commands_[i];
    }

    realtime_motor_command_publisher_->unlockAndPublish();
  }

  return return_type::OK;
}


}  // namespace rosbot_xl_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rosbot_xl_hardware::RosbotXLSystem, hardware_interface::SystemInterface)
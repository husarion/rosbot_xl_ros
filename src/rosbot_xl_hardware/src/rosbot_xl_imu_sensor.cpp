#include "rosbot_xl_hardware/rosbot_xl_imu_sensor.hpp"

#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rosbot_xl_hardware
{
CallbackReturn RosbotXLImuSensor::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  imu_sensor_state_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  node_ = std::make_shared<rclcpp::Node>("imu_sensor_node");

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RosbotXLImuSensor::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(
        StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_sensor_state_[i]));
  }

  return state_interfaces;
}

CallbackReturn RosbotXLImuSensor::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RosbotXLImuSensor"), "Configuring...");

  imu_subscriber_ = node_->create_subscription<Imu>("~/imu", rclcpp::SensorDataQoS(),
                                                    std::bind(&RosbotXLImuSensor::imu_cb, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("RosbotXLImuSensor"), "Successfully configured");

  executor_.add_node(node_);
  executor_thread_ =
      std::make_unique<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotXLImuSensor::on_activate(const rclcpp_lifecycle::State&)
// method where hardware “power” is enabled.
{
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotXLImuSensor::on_deactivate(const rclcpp_lifecycle::State&)
{
  // method, which does the opposite of on_activate.
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

void RosbotXLImuSensor::imu_cb(const std::shared_ptr<Imu> msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "Received imu message");

  if (!subscriber_is_active_)
  {
    RCLCPP_WARN(node_->get_logger(), "Can't accept new messages, subscriber is inactive");
    return;
  }

  received_imu_msg_ptr_.set(std::move(msg));
}

return_type RosbotXLImuSensor::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  std::shared_ptr<Imu> imu_msg;
  received_imu_msg_ptr_.get(imu_msg);

  RCLCPP_DEBUG(rclcpp::get_logger("RosbotXLImuSensor"), "Reading imu state");

  if (!imu_msg)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RosbotXLImuSensor"), "Imu message wasn't yet received");
    // returning ERROR causes controller to freeze
    return return_type::OK;
  }

  imu_sensor_state_[0] = imu_msg->orientation.x;
  imu_sensor_state_[1] = imu_msg->orientation.y;
  imu_sensor_state_[2] = imu_msg->orientation.z;
  imu_sensor_state_[3] = imu_msg->orientation.w;
  imu_sensor_state_[4] = imu_msg->angular_velocity.x;
  imu_sensor_state_[5] = imu_msg->angular_velocity.y;
  imu_sensor_state_[6] = imu_msg->angular_velocity.z;
  imu_sensor_state_[7] = imu_msg->linear_acceleration.x;
  imu_sensor_state_[8] = imu_msg->linear_acceleration.y;
  imu_sensor_state_[9] = imu_msg->linear_acceleration.z;

  return return_type::OK;
}

}  // namespace rosbot_xl_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rosbot_xl_hardware::RosbotXLImuSensor, hardware_interface::SensorInterface)
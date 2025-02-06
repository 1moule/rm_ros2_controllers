//
// Created by guanlin on 25-2-6.
//

#include "rm_ros2_orientation_controller/orientation_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace rm_ros2_orientation_controller
{
OrientationController::OrientationController() : ControllerInterface::ControllerInterface()
{
}

controller_interface::CallbackReturn OrientationController::on_init()
{
  // should have error handling
  imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
  frame_source_ = auto_declare<std::string>("frame_source", frame_source_);
  frame_target_ = auto_declare<std::string>("frame_target", frame_target_);
  imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", imu_interface_types_);

  tf_handler_ = std::make_shared<TfHandler>(get_node());
  tf_broadcaster_ = std::make_shared<TfRtBroadcaster>(get_node());

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OrientationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = { controller_interface::interface_configuration_type::INDIVIDUAL,
                                                        {} };
  return conf;
}

controller_interface::InterfaceConfiguration OrientationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = { controller_interface::interface_configuration_type::INDIVIDUAL,
                                                        {} };
  conf.names.reserve(imu_interface_types_.size());
  for (const auto& interface_type : imu_interface_types_)
  {
    conf.names.push_back(imu_name_ + "/" + interface_type);
  }
  return conf;
}

controller_interface::CallbackReturn OrientationController::on_configure(const rclcpp_lifecycle::State&)
{
  auto imuDataCallback = [this](const std::shared_ptr<sensor_msgs::msg::Imu> msg) -> void {
    imu_data_buffer_.writeFromNonRT(msg);
  };
  imu_data_sub_ =
      get_node()->create_subscription<sensor_msgs::msg::Imu>("data", rclcpp::SystemDefaultsQoS(), imuDataCallback);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OrientationController::on_activate(const rclcpp_lifecycle::State&)
{
  // clear out vectors in case of restart
  imu_state_interface_.clear();
  // assign state interfaces
  for (auto& interface : state_interfaces_)
  {
    if (interface.get_prefix_name() == imu_name_)
    {
      imu_state_interface_.emplace_back(interface);
    }
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type OrientationController::update(const rclcpp::Time& /*time*/,
                                                                const rclcpp::Duration& /*period*/)
{
  return controller_interface::return_type::OK;
}
}  // namespace rm_ros2_orientation_controller

PLUGINLIB_EXPORT_CLASS(rm_ros2_orientation_controller::OrientationController, controller_interface::ControllerInterface)

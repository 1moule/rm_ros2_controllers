//
// Created by guanlin on 25-2-4.
//

#include "rm_ros2_gimbal_controller/gimbal_controller.hpp"

namespace rm_ros2_gimbal_controller {
GimbalController::GimbalController():ControllerInterface::ControllerInterface() {}

controller_interface::CallbackReturn GimbalController::on_init()
{
    // should have error handling
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GimbalController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto & joint_name : joint_names_)
    {
        for (const auto & interface_type : command_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

controller_interface::InterfaceConfiguration GimbalController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto & joint_name : joint_names_)
    {
        for (const auto & interface_type : state_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

controller_interface::CallbackReturn GimbalController::on_configure(const rclcpp_lifecycle::State &)
{
    // cmd gimbal
    auto cmdGimbalCallback =
      [this](const std::shared_ptr<rm_ros2_msgs::msg::GimbalCmd> msg) -> void
      {
          cmd_gimbal_buffer_.writeFromNonRT(msg);
      };
    cmd_gimbal_sub_ =
      get_node()->create_subscription<rm_ros2_msgs::msg::GimbalCmd>(
        "/cmd_chassis", rclcpp::SystemDefaultsQoS(), cmdGimbalCallback);

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalController::on_activate(const rclcpp_lifecycle::State &)
{
    // clear out vectors in case of restart
    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_effort_state_interface_.clear();
    // assign command interfaces
    for (auto & interface : command_interfaces_)
    {
        command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }
    // assign state interfaces
    for (auto & interface : state_interfaces_)
    {
        state_interface_map_[interface.get_interface_name()]->push_back(interface);
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type GimbalController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    cmd_gimbal_=*cmd_gimbal_buffer_.readFromRT();
    return controller_interface::return_type::OK;
}
}
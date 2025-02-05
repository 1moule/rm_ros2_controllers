//
// Created by guanlin on 25-2-5.
//
#include <robot_state_controller/robot_state_controller.hpp>

#include <kdl_parser/kdl_parser.hpp>
// #include <tf2_kdl/tf2_kdl.h>

namespace robot_state_controller
{
controller_interface::CallbackReturn RobotStateController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  // tf_handler_ = std::make_shared<TfHandler>(get_node()->get_name());
  tf_broadcaster_ = std::make_shared<TfRtBroadcaster>(get_node()->get_name());

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotStateController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = { controller_interface::interface_configuration_type::INDIVIDUAL,
                                                        {} };
  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto& joint_name : joint_names_)
  {
    for (const auto& interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration RobotStateController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = { controller_interface::interface_configuration_type::INDIVIDUAL,
                                                        {} };
  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto& joint_name : joint_names_)
  {
    for (const auto& interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::CallbackReturn RobotStateController::on_configure(const rclcpp_lifecycle::State&)
{
  // cmd gimbal
  // auto cmdGimbalCallback = [this](const std::shared_ptr<rm_ros2_msgs::msg::GimbalCmd> msg) -> void {
  //   cmd_gimbal_buffer_.writeFromNonRT(msg);
  // };
  // cmd_gimbal_sub_ = get_node()->create_subscription<rm_ros2_msgs::msg::GimbalCmd>(
  //     "/cmd_gimbal", rclcpp::SystemDefaultsQoS(), cmdGimbalCallback);
  //
  // state_pub_ = get_node()->create_publisher<rm_ros2_msgs::msg::GimbalPosState>(
  //     std::string(get_node()->get_name()) + "/pos_state", rclcpp::SystemDefaultsQoS());
  // rt_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::GimbalPosState>>(state_pub_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotStateController::on_activate(const rclcpp_lifecycle::State&)
{
  // clear out vectors in case of restart
  joint_effort_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  joint_effort_state_interface_.clear();
  // assign command interfaces
  for (auto& interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }
  // assign state interfaces
  for (auto& interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }
  return CallbackReturn::SUCCESS;
}
}  // namespace robot_state_controller
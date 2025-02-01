//
// Created by guanlin on 25-2-1.
//

#ifndef CHASSIS_BASE_HPP
#define CHASSIS_BASE_HPP

#include <controller_interface/controller_interface.hpp>
#include <rm_ros2_msgs/msg/chassis_cmd.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace rm_ros2_chassis_controllers {
struct Command
{
  std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_;
  std::shared_ptr<rm_ros2_msgs::msg::ChassisCmd> cmd_chassis_;
  rclcpp::Time stamp_;
};
class ChassisBase:public controller_interface::ControllerInterface {
public:
  ChassisBase();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_effort_state_interface_;
  std::unordered_map<
  std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
  command_interface_map_ = {
    {"effort", &joint_effort_command_interface_}};
  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
    {"position", &joint_position_state_interface_},
    {"velocity", &joint_velocity_state_interface_},
    {"effort", &joint_effort_state_interface_}};

  rclcpp::Subscription<rm_ros2_msgs::msg::ChassisCmd>::SharedPtr cmd_chassis_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<rm_ros2_msgs::msg::ChassisCmd>> cmd_rt_buffer_;
};
}


#endif //CHASSIS_BASE_HPP

//
// Created by guanlin on 25-2-1.
//

#ifndef CHASSIS_BASE_HPP
#define CHASSIS_BASE_HPP

#include <controller_interface/controller_interface.hpp>
#include <rm_ros2_msgs/msg/chassis_cmd.hpp>
#include <rm_ros2_common/filters/filters.hpp>
#include <rm_ros2_common/ori_tools.hpp>
#include <rm_ros2_common/tf_tools.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <angles/angles.h>

namespace rm_ros2_chassis_controllers {
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
  void raw();
  void follow();
  void recovery();
  void tfVelToBase(const std::string& from);
  void updateOdom(const rclcpp::Time& time);
  virtual void odometry()=0;
  virtual void moveJoint()=0;

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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry> > odom_pub_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry> > odom_pub_nonrt_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<rm_ros2_msgs::msg::ChassisCmd>> cmd_chassis_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_vel_buffer_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> odom_buffer_;
  std::shared_ptr<rm_ros2_msgs::msg::ChassisCmd> cmd_chassis_;
  std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_;
  std::shared_ptr<geometry_msgs::msg::Twist> vel_base_;
  std::shared_ptr<geometry_msgs::msg::Vector3> vel_cmd_;
  rclcpp::Time update_cmd_time_;
  rclcpp::Time last_publish_time_;

  enum
  {
    RAW,
    FOLLOW
  };
  int state_ = RAW;
  double publish_rate_{},timeout_{};
  bool state_changed_ = true;
  std::shared_ptr<RampFilter<double>>ramp_x_{}, ramp_y_{}, ramp_w_{};
  std::shared_ptr<RobotStateHandle> robot_state_handle_;
  std::string follow_source_frame_{}, command_source_frame_{};

};
}


#endif //CHASSIS_BASE_HPP

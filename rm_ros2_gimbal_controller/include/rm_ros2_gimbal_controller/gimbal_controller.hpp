//
// Created by guanlin on 25-2-4.
//

#ifndef GIMBAL_CONTROLLER_HPP
#define GIMBAL_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <rm_ros2_msgs/msg/gimbal_cmd.hpp>
#include <rm_ros2_msgs/msg/gimbal_pos_state.hpp>
#include <rm_ros2_common/tools/tf_tools.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <urdf/urdf/model.h>
#include <control_toolbox/pid_ros.hpp>

namespace rm_ros2_gimbal_controller
{
class GimbalController : public controller_interface::ControllerInterface
{
public:
  GimbalController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  void rate(const rclcpp::Time& time, const rclcpp::Duration& period);

  void traj(const rclcpp::Time& time);

  void setDes(const rclcpp::Time& time, double yaw_des, double pitch_des);

  static bool setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                              const urdf::JointConstSharedPtr& joint_urdf);

  void moveJoint(const rclcpp::Time& time, const rclcpp::Duration& period) const;

  //  hardware interface
  std::string imu_name_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::vector<std::string> imu_interface_types_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > joint_velocity_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > joint_effort_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > imu_state_interface_;
  std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >*>
      command_interface_map_ = { { "effort", &joint_effort_command_interface_ } };
  std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >*>
      state_interface_map_ = { { "position", &joint_position_state_interface_ },
                               { "velocity", &joint_velocity_state_interface_ },
                               { "effort", &joint_effort_state_interface_ } };

  //  ROS interface
  rclcpp::Subscription<rm_ros2_msgs::msg::GimbalCmd>::SharedPtr cmd_gimbal_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<rm_ros2_msgs::msg::GimbalCmd> > cmd_gimbal_buffer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::GimbalPosState> > rt_state_pub_;
  std::shared_ptr<rclcpp::Publisher<rm_ros2_msgs::msg::GimbalPosState> > state_pub_;
  std::shared_ptr<rm_ros2_msgs::msg::GimbalCmd> cmd_gimbal_;
  std::shared_ptr<control_toolbox::PidROS> pid_pos_yaw_, pid_vel_yaw_;
  geometry_msgs::msg::TransformStamped odom2gimbal_des_, odom2pitch_, odom2base_, last_odom2base_;
  std::vector<urdf::JointConstSharedPtr> joint_urdf_;

  enum
  {
    RATE,
    TRACK,
    DIRECT,
    TRAJ
  };

  int state_ = RATE;
  double publish_rate_{};
  bool state_changed_{};
  bool start_ = true;
  bool pitch_des_in_limit_{};
  bool yaw_des_in_limit_{};
  bool has_imu_ = false;
  std::shared_ptr<TfHandler> tf_handler_;
  std::shared_ptr<TfRtBroadcaster> tf_broadcaster_;
  std::string gimbal_des_frame_id_{};
};
}  // namespace rm_ros2_gimbal_controller
#endif  // GIMBAL_CONTROLLER_HPP

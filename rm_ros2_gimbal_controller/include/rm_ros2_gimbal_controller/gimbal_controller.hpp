//
// Created by guanlin on 25-2-4.
//

#ifndef GIMBAL_CONTROLLER_HPP
#define GIMBAL_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <nav_msgs/nav_msgs/msg/odometry.hpp>
#include <rm_ros2_msgs/msg/gimbal_pos_state.hpp>
#include <rm_ros2_msgs/msg/gimbal_cmd.hpp>
#include <rm_ros2_msgs/msg/track_data.hpp>
#include <rm_ros2_common/tools/tf_tools.hpp>
#include <rm_ros2_common/decision/bullet_solver/bullet_solver.hpp>
#include <rm_ros2_common/decision/trajectory_planner.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <semantic_components/imu_sensor.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <urdf/urdf/model.h>

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
  void track(const rclcpp::Time& time);
  void traj(const rclcpp::Time& time);
  void setDes(const rclcpp::Time& time, double yaw_des, double pitch_des);
  static bool setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des, double temp,
                              const urdf::JointConstSharedPtr& joint_urdf, tf2::Quaternion& base2new_des);
  double frictionFeedforward() const;
  void moveJoint(const rclcpp::Time& time, const rclcpp::Duration& period);

  //  hardware interface
  std::string imu_name_;
  std::shared_ptr<semantic_components::IMUSensor> imu_sensor_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_effort_state_interface_;
  std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
      command_interface_map_ = { { "effort", &joint_effort_command_interface_ } };
  std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
      state_interface_map_ = { { "position", &joint_position_state_interface_ },
                               { "velocity", &joint_velocity_state_interface_ },
                               { "effort", &joint_effort_state_interface_ } };

  //  ROS interface
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::GimbalPosState>> pitch_rt_pos_state_pub_,
      yaw_rt_pos_state_pub_;
  std::shared_ptr<rclcpp::Publisher<rm_ros2_msgs::msg::GimbalPosState>> pitch_pos_state_pub_, yaw_pos_state_pub_;
  std::shared_ptr<rm_ros2_msgs::msg::GimbalCmd> cmd_gimbal_;
  std::shared_ptr<rm_ros2_msgs::msg::TrackData> track_data_;
  std::shared_ptr<nav_msgs::msg::Odometry> odom_;
  std::shared_ptr<control_toolbox::PidROS> pid_pos_pitch_, pid_vel_pitch_, pid_pos_yaw_, pid_vel_yaw_;
  std::shared_ptr<TfHandler> tf_handler_;
  std::shared_ptr<TfRtBroadcaster> tf_broadcaster_;
  std::shared_ptr<geometry_msgs::msg::Twist> chassis_vel_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Subscription<rm_ros2_msgs::msg::GimbalCmd>::SharedPtr cmd_gimbal_sub_;
  rclcpp::Subscription<rm_ros2_msgs::msg::TrackData>::SharedPtr track_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<rm_ros2_msgs::msg::GimbalCmd>> cmd_gimbal_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<rm_ros2_msgs::msg::TrackData>> track_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<nav_msgs::msg::Odometry>> odom_buffer_;
  geometry_msgs::msg::TransformStamped odom2gimbal_des_, odom2pitch_, odom2base_, last_odom2base_;
  std::vector<urdf::JointConstSharedPtr> joint_urdf_;

  enum
  {
    RATE,
    TRACK,
    TRAJ
  };
  int state_ = RATE;
  double publish_rate_{};
  double b_{}, r_{}, h_{};
  bool state_changed_{};
  bool start_ = true;
  bool pitch_des_in_limit_ = false;
  bool yaw_des_in_limit_ = false;
  bool has_imu_ = false;
  std::string gimbal_des_frame_id_{};
  std::shared_ptr<bullet_solver::BulletSolver> bullet_solver_;
  std::shared_ptr<NonlinearTrackingDifferentiator<double>> tracking_differentiator_;
};
}  // namespace rm_ros2_gimbal_controller
#endif  // GIMBAL_CONTROLLER_HPP

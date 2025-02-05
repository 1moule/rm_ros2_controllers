//
// Created by guanlin on 25-2-5.
//

#ifndef ROBOT_STATE_CONTROLLER_HPP
#define ROBOT_STATE_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <rm_ros2_common/tools/tf_tools.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <realtime_tools/realtime_buffer.hpp>
#include <kdl/tree.hpp>
#include <urdf/urdf/model.h>

namespace robot_state_controller
{
class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, std::string p_root, std::string p_tip)
    : segment(p_segment), root(std::move(p_root)), tip(std::move(p_tip))
  {
  }

  KDL::Segment segment{};
  std::string root, tip;
};

class RobotStateController : public controller_interface::ControllerInterface
{
public:
  RobotStateController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  void addChildren(KDL::SegmentMap::const_iterator segment);

  //  hardware interface
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

  urdf::Model model_{};
  bool use_tf_static_{};
  double publish_rate_{};
  rclcpp::Time last_update_;
  rclcpp::Time last_publish_time_;

  std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> jnt_states_;
  std::map<std::string, SegmentPair> segments_, segments_fixed_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<TfRtBroadcaster> tf_broadcaster_;
  std::shared_ptr<StaticTfRtBroadcaster> static_tf_broadcaster_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<tf2_msgs::msg::TFMessage>> tf_msg_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<tf2_msgs::msg::TFMessage>> tf_static_msg_;
};
}  // namespace robot_state_controller
#endif  // ROBOT_STATE_CONTROLLER_HPP

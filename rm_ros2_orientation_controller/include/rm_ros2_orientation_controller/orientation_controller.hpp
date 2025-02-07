//
// Created by guanlin on 25-2-6.
//

#ifndef ORIENTATION_CONTROLLER_HPP
#define ORIENTATION_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rm_ros2_common/tools/tf_tools.hpp>
#include <semantic_components/imu_sensor.hpp>

namespace rm_ros2_orientation_controller
{
class OrientationController final : public controller_interface::ControllerInterface
{
public:
  OrientationController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  bool getTransform(geometry_msgs::msg::TransformStamped& source2target, const double x, const double y, const double z,
                    const double w);

  // hardware interface
  std::string imu_name_;
  std::shared_ptr<semantic_components::IMUSensor> imu_sensor_;

  // ROS interface
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
  rclcpp::Time last_imu_update_time_;
  std::shared_ptr<TfHandler> tf_handler_;
  std::shared_ptr<TfRtBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped source2target_msg_;

  double timeout_{};
  std::string frame_source_, frame_target_;
};
}  // namespace rm_ros2_orientation_controller
#endif  // ORIENTATION_CONTROLLER_HPP

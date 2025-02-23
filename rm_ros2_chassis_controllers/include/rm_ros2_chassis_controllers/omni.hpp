//
// Created by guanlin on 25-2-2.
//

#ifndef OMNI_HPP
#define OMNI_HPP

#include "rm_ros2_chassis_controllers/chassis_base.hpp"
#include <control_toolbox/pid_ros.hpp>
#include <Eigen/Dense>
#include <vector>

namespace rm_ros2_chassis_controllers
{
class OmniController final : public ChassisBase
{
public:
  OmniController();
  controller_interface::CallbackReturn on_init() override;

private:
  void moveJoint(const rclcpp::Duration& period) override;
  void odometry() override;
  std::vector<std::shared_ptr<control_toolbox::PidROS>> pid_joints_;
  std::vector<std::string> wheel_names_;
  Eigen::MatrixXd chassis2joints_;
  double radius_{}, p_{};
};
}  // namespace rm_ros2_chassis_controllers

#endif  // OMNI_HPP

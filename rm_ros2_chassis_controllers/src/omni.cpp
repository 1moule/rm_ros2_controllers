//
// Created by guanlin on 25-2-2.
//

#include "rm_ros2_chassis_controllers/omni.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace rm_ros2_chassis_controllers
{
OmniController::OmniController() : ChassisBase::ChassisBase()
{
}
controller_interface::CallbackReturn OmniController::on_init()
{
  ChassisBase::on_init();
  left_front_wheel_pos_ = auto_declare<std::vector<double>>("left_front_wheel_pos", left_front_wheel_pos_);
  right_front_wheel_pos_ = auto_declare<std::vector<double>>("right_front_wheel_pos", right_front_wheel_pos_);
  left_back_wheel_pos_ = auto_declare<std::vector<double>>("left_back_wheel_pos", left_back_wheel_pos_);
  right_back_wheel_pos_ = auto_declare<std::vector<double>>("right_back_wheel_pos", right_back_wheel_pos_);
  wheels_pos_.push_back(left_front_wheel_pos_);
  wheels_pos_.push_back(right_front_wheel_pos_);
  wheels_pos_.push_back(left_back_wheel_pos_);
  wheels_pos_.push_back(right_back_wheel_pos_);
  roller_angles_ = auto_declare<std::vector<double>>("roller_angles", roller_angles_);
  radius_ = auto_declare<double>("wheel_radius", radius_);
  p_ = auto_declare<double>("p", p_);
  chassis2joints_.resize(wheels_pos_.size(), 3);
  for (size_t i = 0; i < wheels_pos_.size(); i++)
  {
    // Ref: Modern Robotics, Chapter 13.2: Omnidirectional Wheeled Mobile Robots
    Eigen::MatrixXd direction(1, 2), in_wheel(2, 2), in_chassis(2, 3);
    double beta = wheels_pos_[i][2];
    double roller_angle = roller_angles_[i];
    direction << 1, tan(roller_angle);
    in_wheel << cos(beta), sin(beta), -sin(beta), cos(beta);
    in_chassis << -wheels_pos_[i][1], 1., 0., wheels_pos_[i][0], 0., 1.;
    Eigen::MatrixXd chassis2joint = 1. / radius_ * direction * in_wheel * in_chassis;
    chassis2joints_.block<1, 3>(i, 0) = chassis2joint;
  }
  return CallbackReturn::SUCCESS;
}

void OmniController::moveJoint()
{
  Eigen::Vector3d vel_chassis;
  vel_chassis << vel_cmd_->z, vel_cmd_->x, vel_cmd_->y;
  Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;
  for (size_t i = 0; i < joint_effort_command_interface_.size(); i++)
  {
    joint_effort_command_interface_[i].get().set_value(
        p_ * (vel_joints(i) - joint_velocity_state_interface_[i].get().get_value()));
  }
}

void OmniController::odometry()
{
  Eigen::VectorXd vel_joints(wheels_pos_.size());
  for (size_t i = 0; i < wheels_pos_.size(); i++)
    vel_joints[i] = joint_velocity_state_interface_[i].get().get_value();
  Eigen::Vector3d vel_chassis =
      (chassis2joints_.transpose() * chassis2joints_).inverse() * chassis2joints_.transpose() * vel_joints;
  vel_base_->angular.z = vel_chassis(0);
  vel_base_->linear.x = vel_chassis(1);
  vel_base_->linear.y = vel_chassis(2);
}
}  // namespace rm_ros2_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_ros2_chassis_controllers::OmniController, controller_interface::ControllerInterface)

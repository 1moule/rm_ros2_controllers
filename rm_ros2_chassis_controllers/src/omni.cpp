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
  wheel_names_ = auto_declare<std::vector<std::string>>("wheels.name", wheel_names_);
  radius_ = auto_declare<double>("wheels.radius", radius_);
  p_ = auto_declare<double>("wheels.p", p_);
  chassis2joints_.resize(wheel_names_.size(), 3);
  size_t i = 0;
  for (const auto& wheel_name : wheel_names_)
  {
    std::cout << wheel_name << std::endl;
    // Ref: Modern Robotics, Chapter 13.2: Omnidirectional Wheeled Mobile Robots
    Eigen::MatrixXd direction(1, 2), in_wheel(2, 2), in_chassis(2, 3);
    std::vector<double> wheel_pos = auto_declare<std::vector<double>>("wheels." + wheel_name + ".pos", { 0., 0., 0. });
    double beta = wheel_pos[2];
    double roller_angle = auto_declare<double>("wheels." + wheel_name + ".roller_angle", 0.);
    direction << 1, tan(roller_angle);
    in_wheel << cos(beta), sin(beta), -sin(beta), cos(beta);
    in_chassis << -wheel_pos[1], 1., 0., wheel_pos[0], 0., 1.;
    Eigen::MatrixXd chassis2joint = 1. / radius_ * direction * in_wheel * in_chassis;
    chassis2joints_.block<1, 3>(i, 0) = chassis2joint;

    i++;
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
  Eigen::VectorXd vel_joints(wheel_names_.size());
  for (size_t i = 0; i < wheel_names_.size(); i++)
    vel_joints[i] = joint_velocity_state_interface_[i].get().get_value();
  Eigen::Vector3d vel_chassis =
      (chassis2joints_.transpose() * chassis2joints_).inverse() * chassis2joints_.transpose() * vel_joints;
  vel_base_->angular.z = vel_chassis(0);
  vel_base_->linear.x = vel_chassis(1);
  vel_base_->linear.y = vel_chassis(2);
}
}  // namespace rm_ros2_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_ros2_chassis_controllers::OmniController, controller_interface::ControllerInterface)

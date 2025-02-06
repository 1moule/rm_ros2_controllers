//
// Created by guanlin on 24-12-25.
//

#include "rm_ros2_chassis_controllers/chassis_base.hpp"
#include <rm_ros2_common/tools/ori_tools.hpp>
#include <angles/angles.h>

namespace rm_ros2_chassis_controllers
{
ChassisBase::ChassisBase() : controller_interface::ControllerInterface()
{
}
controller_interface::CallbackReturn ChassisBase::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
  publish_rate_ = auto_declare<double>("publish_rate", 100.0);
  timeout_ = auto_declare<double>("timeout", 0.1);
  max_odom_vel_ = auto_declare<double>("max_odom_vel", 999.0);
  enable_odom_tf_ = auto_declare<bool>("enable_odom_tf", false);
  publish_odom_tf_ = auto_declare<bool>("publish_odom_tf", false);

  cmd_chassis_ = std::shared_ptr<rm_ros2_msgs::msg::ChassisCmd>();
  cmd_vel_ = std::make_shared<geometry_msgs::msg::Twist>();
  vel_base_ = std::make_shared<geometry_msgs::msg::Twist>();
  vel_cmd_ = std::make_shared<geometry_msgs::msg::Vector3>();

  ramp_x_ = std::make_shared<RampFilter<double>>(0, 0.001);
  ramp_y_ = std::make_shared<RampFilter<double>>(0, 0.001);
  ramp_w_ = std::make_shared<RampFilter<double>>(0, 0.001);
  tf_handler_ = std::make_shared<TfHandler>(get_node());
  tf_broadcaster_ = std::make_shared<TfRtBroadcaster>(get_node());

  pid_follow_ = std::make_shared<control_toolbox::PidROS>(get_node()->get_node_base_interface(),
                                                          get_node()->get_node_logging_interface(),
                                                          get_node()->get_node_parameters_interface(),
                                                          get_node()->get_node_topics_interface(), "pid_follow", true);
  pid_follow_->initPid();

  last_publish_time_ = get_node()->get_clock()->now();
  update_cmd_time_ = get_node()->get_clock()->now();

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChassisBase::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration ChassisBase::state_interface_configuration() const
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

controller_interface::CallbackReturn ChassisBase::on_configure(const rclcpp_lifecycle::State&)
{
  // cmd chassis
  auto cmdChassisCallback = [this](const std::shared_ptr<rm_ros2_msgs::msg::ChassisCmd> msg) -> void {
    cmd_chassis_buffer_.writeFromNonRT(msg);
  };
  cmd_chassis_sub_ = get_node()->create_subscription<rm_ros2_msgs::msg::ChassisCmd>(
      "/cmd_chassis", rclcpp::SystemDefaultsQoS(), cmdChassisCallback);

  // cmd vel
  auto cmdVelCallback = [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
    cmd_vel_buffer_.writeFromNonRT(msg);
    update_cmd_time_ = get_node()->get_clock()->now();
  };
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS(),
                                                                            cmdVelCallback);

  odom_pub_nonrt_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());
  odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_nonrt_);
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = "base_link";
  odom_pub_->msg_.twist.covariance = { 0.01, 0., 0.,   0., 0.,   0., 0., 0.01, 0., 0.,   0., 0.,
                                       0.,   0., 0.01, 0., 0.,   0., 0., 0.,   0., 0.01, 0., 0.,
                                       0.,   0., 0.,   0., 0.01, 0., 0., 0.,   0., 0.,   0., 0.01 };
  if (enable_odom_tf_)
  {
    odom2base_.header.frame_id = "odom";
    odom2base_.header.stamp = get_node()->get_clock()->now();
    odom2base_.child_frame_id = "base_link";
    odom2base_.transform.rotation.w = 1;
    tf_broadcaster_->sendTransform(odom2base_);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisBase::on_activate(const rclcpp_lifecycle::State&)
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

controller_interface::return_type ChassisBase::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  cmd_chassis_ = *cmd_chassis_buffer_.readFromRT();
  cmd_vel_ = *cmd_vel_buffer_.readFromRT();
  if ((time - update_cmd_time_) > rclcpp::Duration::from_seconds(timeout_))
  {
    vel_cmd_->x = 0.;
    vel_cmd_->y = 0.;
    vel_cmd_->z = 0.;
  }
  else
  {
    if (cmd_chassis_ != nullptr)
    {
      ramp_x_->setAcc(cmd_chassis_->accel.linear.x);
      ramp_y_->setAcc(cmd_chassis_->accel.linear.y);
      ramp_w_->setAcc(cmd_chassis_->accel.angular.z);
    }
    if (cmd_vel_ != nullptr)
    {
      ramp_x_->input(cmd_vel_->linear.x);
      ramp_y_->input(cmd_vel_->linear.y);
      vel_cmd_->x = ramp_x_->output();
      vel_cmd_->y = ramp_y_->output();
      vel_cmd_->z = cmd_vel_->angular.z;
    }
  }
  if (cmd_chassis_ != nullptr)
  {
    if (cmd_chassis_->follow_source_frame.empty())
      follow_source_frame_ = "yaw";
    else
      follow_source_frame_ = cmd_chassis_->follow_source_frame;
    if (cmd_chassis_->command_source_frame.empty())
      command_source_frame_ = "yaw";
    else
      command_source_frame_ = cmd_chassis_->command_source_frame;
    if (state_ != cmd_chassis_->mode)
    {
      state_ = cmd_chassis_->mode;
      state_changed_ = true;
    }
    switch (state_)
    {
      case RAW:
        raw();
        break;
      case FOLLOW:
        follow(period);
        break;
    }
    ramp_w_->input(vel_cmd_->z);
    vel_cmd_->z = ramp_w_->output();
  }
  updateOdom(time, period);
  moveJoint();

  return controller_interface::return_type::OK;
}

void ChassisBase::raw()
{
  if (state_changed_)
  {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "[Chassis] Enter RAW");

    recovery();
  }
  tfVelToBase(command_source_frame_);
}

void ChassisBase::follow(const rclcpp::Duration& period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "[Chassis] Enter FOLLOW");

    recovery();
  }
  tfVelToBase(command_source_frame_);
  try
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(tf_handler_->lookupTransform("base_link", follow_source_frame_).transform.rotation, roll, pitch, yaw);
    double follow_error = angles::shortest_angular_distance(yaw, 0);
    vel_cmd_->z = pid_follow_->computeCommand(-follow_error, period) + cmd_chassis_->follow_vel_des;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN_ONCE(get_node()->get_logger(), "%s", ex.what());
  }
}

void ChassisBase::recovery()
{
  ramp_x_->clear(vel_cmd_->x);
  ramp_y_->clear(vel_cmd_->y);
  ramp_w_->clear(vel_cmd_->z);
}

void ChassisBase::tfVelToBase(const std::string& from)
{
  try
  {
    tf2::doTransform(*vel_cmd_, *vel_cmd_, tf_handler_->lookupTransform("base_link", from));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN_ONCE(get_node()->get_logger(), "%s", ex.what());
  }
}

void ChassisBase::updateOdom(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  odometry();
  if (enable_odom_tf_)
  {
    geometry_msgs::msg::Vector3 linear_vel_odom, angular_vel_odom;
    try
    {
      odom2base_ = tf_handler_->lookupTransform("odom", "base_link");
    }
    catch (tf2::TransformException& ex)
    {
      tf_broadcaster_->sendTransform(odom2base_);
      RCLCPP_WARN_ONCE(get_node()->get_logger(), "%s", ex.what());
      return;
    }
    odom2base_.header.stamp = time;
    // integral vel to pos and angle
    tf2::doTransform(vel_base_->linear, linear_vel_odom, odom2base_);
    tf2::doTransform(vel_base_->angular, angular_vel_odom, odom2base_);
    double length =
        std::sqrt(std::pow(linear_vel_odom.x, 2) + std::pow(linear_vel_odom.y, 2) + std::pow(linear_vel_odom.z, 2));
    if (length < max_odom_vel_)
    {
      // avoid nan vel
      odom2base_.transform.translation.x += linear_vel_odom.x * period.seconds();
      odom2base_.transform.translation.y += linear_vel_odom.y * period.seconds();
      odom2base_.transform.translation.z += linear_vel_odom.z * period.seconds();
    }
    length =
        std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) + std::pow(angular_vel_odom.z, 2));
    if (length > 0.001)
    {  // avoid nan quat
      tf2::Quaternion odom2base_quat, trans_quat;
      tf2::fromMsg(odom2base_.transform.rotation, odom2base_quat);
      trans_quat.setRotation(tf2::Vector3(angular_vel_odom.x / length, angular_vel_odom.y / length,
                                          angular_vel_odom.z / length),
                             length * period.seconds());
      odom2base_quat = trans_quat * odom2base_quat;
      odom2base_quat.normalize();
      odom2base_.transform.rotation = tf2::toMsg(odom2base_quat);
    }
  }
  if (publish_rate_ > 0.0 && last_publish_time_ + rclcpp::Duration::from_seconds(1.0 / publish_rate_) < time)
  {
    if (odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.twist.twist.linear.x = vel_base_->linear.x;
      odom_pub_->msg_.twist.twist.linear.y = vel_base_->linear.y;
      odom_pub_->msg_.twist.twist.angular.z = vel_base_->angular.z;
      odom_pub_->unlockAndPublish();
    }
    if (enable_odom_tf_ && publish_odom_tf_)
      tf_broadcaster_->sendTransform(odom2base_);
    last_publish_time_ = time;
  }
}
}  // namespace rm_ros2_chassis_controllers

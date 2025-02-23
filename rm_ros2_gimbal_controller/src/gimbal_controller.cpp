//
// Created by guanlin on 25-2-4.
//

#include "rm_ros2_gimbal_controller/gimbal_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rm_ros2_common/tools/ori_tools.hpp>
#include <angles/angles/angles.h>

namespace rm_ros2_gimbal_controller
{
GimbalController::GimbalController() : ControllerInterface::ControllerInterface()
{
}

controller_interface::CallbackReturn GimbalController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
  publish_rate_ = auto_declare<double>("publish_rate", 100.0);
  b_ = auto_declare<double>("b", 0.0);
  r_ = auto_declare<double>("r", 100.0);
  h_ = auto_declare<double>("h", 0.001);
  if (get_node()->has_parameter("imu_name"))
  {
    has_imu_ = true;
    imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
    imu_sensor_ = std::make_shared<semantic_components::IMUSensor>(semantic_components::IMUSensor(imu_name_));
  }

  cmd_gimbal_ = std::make_shared<rm_ros2_msgs::msg::GimbalCmd>();
  odom_ = std::make_shared<nav_msgs::msg::Odometry>();

  pid_pos_pitch_ = std::make_shared<control_toolbox::PidROS>(
      get_node()->get_node_base_interface(), get_node()->get_node_logging_interface(),
      get_node()->get_node_parameters_interface(), get_node()->get_node_topics_interface(), "pitch.pid_pos", true);
  pid_vel_pitch_ = std::make_shared<control_toolbox::PidROS>(
      get_node()->get_node_base_interface(), get_node()->get_node_logging_interface(),
      get_node()->get_node_parameters_interface(), get_node()->get_node_topics_interface(), "pitch.pid_vel", true);
  pid_pos_yaw_ = std::make_shared<control_toolbox::PidROS>(
      get_node()->get_node_base_interface(), get_node()->get_node_logging_interface(),
      get_node()->get_node_parameters_interface(), get_node()->get_node_topics_interface(), "yaw.pid_pos", true);
  pid_vel_yaw_ = std::make_shared<control_toolbox::PidROS>(
      get_node()->get_node_base_interface(), get_node()->get_node_logging_interface(),
      get_node()->get_node_parameters_interface(), get_node()->get_node_topics_interface(), "yaw.pid_vel", true);
  pid_pos_pitch_->initPid();
  pid_vel_pitch_->initPid();
  pid_pos_yaw_->initPid();
  pid_vel_yaw_->initPid();

  tf_handler_ = std::make_shared<TfHandler>(get_node());
  tf_broadcaster_ = std::make_shared<TfRtBroadcaster>(get_node());
  chassis_vel_ = std::make_shared<geometry_msgs::msg::Twist>();
  bullet_solver_ = std::make_shared<bullet_solver::BulletSolver>(get_node());
  tracking_differentiator_ = std::make_shared<NonlinearTrackingDifferentiator<double>>(r_, h_);

  // get URDF info about joint
  urdf::Model urdf;
  std::string urdf_string;
  get_node()->get_parameter_or<std::string>("robot_description", urdf_string, "");
  if (!urdf.initString(urdf_string))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
    return CallbackReturn::ERROR;
  }
  for (const auto& name : joint_names_)
  {
    auto joint_urdf = urdf.getJoint(name);
    if (!joint_urdf)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not find %s in urdf", name.c_str());
      return CallbackReturn::ERROR;
    }
    else
    {
      joint_urdf_.push_back(joint_urdf);
    }
  }

  gimbal_des_frame_id_ = joint_urdf_[0]->child_link_name + "_des";
  odom2gimbal_des_.header.frame_id = "odom";
  odom2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.;
  odom2pitch_.header.frame_id = "odom";
  odom2pitch_.child_frame_id = joint_urdf_[0]->child_link_name;
  odom2pitch_.transform.rotation.w = 1.;
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = joint_urdf_[1]->parent_link_name;
  odom2base_.transform.rotation.w = 1.;

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GimbalController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration GimbalController::state_interface_configuration() const
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
  if (has_imu_)
  {
    for (const auto& interface : imu_sensor_->get_state_interface_names())
      conf.names.push_back(interface);
  }

  return conf;
}

controller_interface::CallbackReturn GimbalController::on_configure(const rclcpp_lifecycle::State&)
{
  // subscriber
  auto cmdGimbalCallback = [this](const std::shared_ptr<rm_ros2_msgs::msg::GimbalCmd> msg) -> void {
    cmd_gimbal_buffer_.writeFromNonRT(msg);
  };
  cmd_gimbal_sub_ = get_node()->create_subscription<rm_ros2_msgs::msg::GimbalCmd>(
      "/cmd_gimbal", rclcpp::SystemDefaultsQoS(), cmdGimbalCallback);
  auto odomCallback = [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) -> void {
    odom_buffer_.writeFromNonRT(msg);
  };
  odom_sub_ =
      get_node()->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS(), odomCallback);
  auto trackCallback = [this](const std::shared_ptr<rm_ros2_msgs::msg::TrackData> msg) -> void {
    track_buffer_.writeFromNonRT(msg);
  };
  track_sub_ = get_node()->create_subscription<rm_ros2_msgs::msg::TrackData>("/track", rclcpp::SystemDefaultsQoS(),
                                                                             trackCallback);

  // publisher
  pitch_pos_state_pub_ = get_node()->create_publisher<rm_ros2_msgs::msg::GimbalPosState>(
      std::string(get_node()->get_name()) + "/pitch/pos_state", rclcpp::SystemDefaultsQoS());
  yaw_pos_state_pub_ = get_node()->create_publisher<rm_ros2_msgs::msg::GimbalPosState>(
      std::string(get_node()->get_name()) + "/yaw/pos_state", rclcpp::SystemDefaultsQoS());
  pitch_rt_pos_state_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::GimbalPosState>>(pitch_pos_state_pub_);
  yaw_rt_pos_state_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::GimbalPosState>>(yaw_pos_state_pub_);

  // dynamic reconfigure
  setParameterEventCallBack();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalController::on_activate(const rclcpp_lifecycle::State&)
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
    if ((has_imu_ && interface.get_prefix_name() != imu_name_) || !has_imu_)
      state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }
  if (has_imu_)
    imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GimbalController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  cmd_gimbal_ = *cmd_gimbal_buffer_.readFromRT();
  track_data_ = *track_buffer_.readFromRT();
  odom_ = *odom_buffer_.readFromRT();
  try
  {
    odom2pitch_ = tf_handler_->lookupTransform("odom", joint_urdf_[0]->child_link_name, time);
    odom2base_ = tf_handler_->lookupTransform("odom", joint_urdf_[1]->parent_link_name, time);
    if (odom_ != nullptr)
    {
      tf2::doTransform(odom_->twist.twist.linear, chassis_vel_->linear, odom2base_);
      tf2::doTransform(odom_->twist.twist.angular, chassis_vel_->angular, odom2base_);
    }
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN_ONCE(get_node()->get_logger(), "%s", ex.what());
  }
  if (cmd_gimbal_ != nullptr)
  {
    if (state_ != cmd_gimbal_->mode)
    {
      state_ = cmd_gimbal_->mode;
      state_changed_ = true;
    }
    switch (state_)
    {
      case RATE:
        rate(time, period);
        break;
      case TRACK:
        track(time);
        break;
      case TRAJ:
        traj(time);
        break;
      default:
        break;
    }
  }
  moveJoint(time, period);
  return controller_interface::return_type::OK;
}

void GimbalController::rate(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "[Gimbal] Enter RATE");
    if (start_)
    {
      odom2gimbal_des_.transform.rotation = odom2pitch_.transform.rotation;
      odom2gimbal_des_.header.stamp = time;
      tf_handler_->setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
      start_ = false;
    }
  }
  else
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time, yaw + period.seconds() * cmd_gimbal_->rate_yaw, pitch + period.seconds() * cmd_gimbal_->rate_pitch);
  }
}

void GimbalController::track(const rclcpp::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "[Gimbal] Enter TRACK");
  }
  if (track_data_ != nullptr)
  {
    double roll_real, pitch_real, yaw_real;
    quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
    // double yaw_compute = yaw_real;
    // double pitch_compute = -pitch_real;
    geometry_msgs::msg::Point target_pos = track_data_->position;
    geometry_msgs::msg::Vector3 target_vel = track_data_->velocity;
    try
    {
      if (!track_data_->header.frame_id.empty())
      {
        geometry_msgs::msg::TransformStamped transform =
            tf_handler_->lookupTransform("odom", track_data_->header.frame_id, track_data_->header.stamp);
        tf2::doTransform(target_pos, target_pos, transform);
        tf2::doTransform(target_vel, target_vel, transform);
      }
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN_ONCE(get_node()->get_logger(), "%s", ex.what());
    }

    double yaw = fmod(track_data_->yaw + track_data_->v_yaw * ((time - track_data_->header.stamp).seconds()), M_2_PI);
    target_pos.x += target_vel.x * (time - track_data_->header.stamp).seconds() - odom2pitch_.transform.translation.x;
    target_pos.y += target_vel.y * (time - track_data_->header.stamp).seconds() - odom2pitch_.transform.translation.y;
    target_pos.z += target_vel.z * (time - track_data_->header.stamp).seconds() - odom2pitch_.transform.translation.z;
    target_vel.x -= chassis_vel_->linear.x;
    target_vel.y -= chassis_vel_->linear.y;
    target_vel.z -= chassis_vel_->linear.z;
    bullet_solver_->selectTarget(target_pos, target_vel, cmd_gimbal_->bullet_speed, yaw, track_data_->v_yaw,
                                 track_data_->radius_1, track_data_->radius_2, track_data_->dz, track_data_->id);
    // if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
    // {
    //   if (error_pub_->trylock())
    //   {
    //     double error =
    //         bullet_solver_->getGimbalError(target_pos, target_vel, track_data_.yaw, track_data_.v_yaw,
    //                                        track_data_.radius_1, track_data_.radius_2, track_data_.dz,
    //                                        track_data_.armors_num, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed);
    //     error_pub_->msg_.stamp = time;
    //     error_pub_->msg_.error = solve_success ? error : 1.0;
    //     error_pub_->unlockAndPublish();
    //   }
    //   bullet_solver_->bulletModelPub(odom2pitch_, time);
    //   last_publish_time_ = time;
    // }
    if (bullet_solver_->solve())
      setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch());
    else
    {
      odom2gimbal_des_.header.stamp = time;
      tf_handler_->setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
    }
  }
}

void GimbalController::traj(const rclcpp::Time& time)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "[Gimbal] Enter TRAJ");
  }
  setDes(time, cmd_gimbal_->traj_yaw, cmd_gimbal_->traj_pitch);
}

void GimbalController::setDes(const rclcpp::Time& time, double yaw_des, double pitch_des)
{
  tf2::Quaternion odom2base, odom2gimbal_des, base2new_des;
  tf2::fromMsg(odom2base_.transform.rotation, odom2base);
  odom2gimbal_des.setRPY(0, pitch_des, yaw_des);
  tf2::Quaternion base2gimbal_des = odom2base.inverse() * odom2gimbal_des;
  double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
  quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw);
  double pitch_real_des, yaw_real_des;
  pitch_des_in_limit_ = setDesIntoLimit(pitch_real_des, pitch_des, base2gimbal_current_des_pitch,
                                        base2gimbal_current_des_yaw, joint_urdf_[0], base2new_des);
  yaw_des_in_limit_ = setDesIntoLimit(yaw_real_des, yaw_des, base2gimbal_current_des_yaw, base2gimbal_current_des_pitch,
                                      joint_urdf_[1], base2new_des);
  if (!pitch_des_in_limit_ || !yaw_des_in_limit_)
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_real_des, yaw_real_des);
  createQuaternionMsgFromRollPitchYaw(odom2gimbal_des_.transform.rotation, 0., pitch_real_des, yaw_real_des);
  odom2gimbal_des_.header.stamp = time;
  tf_handler_->setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
}

bool GimbalController::setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                                       double temp, const urdf::JointConstSharedPtr& joint_urdf,
                                       tf2::Quaternion& base2new_des)
{
  double upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16;
  double lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16;
  if ((base2gimbal_current_des <= upper_limit && base2gimbal_current_des >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des) >= lower_limit))
  {
    real_des = current_des;
    return true;
  }
  const double new_des = std::abs(angles::shortest_angular_distance(base2gimbal_current_des, upper_limit)) <
                                 std::abs(angles::shortest_angular_distance(base2gimbal_current_des, lower_limit)) ?
                             upper_limit :
                             lower_limit;
  if (joint_urdf->name.find("pitch") != std::string::npos)
    base2new_des.setRPY(0, new_des, temp);
  else
    base2new_des.setRPY(0, temp, new_des);
  return false;
}

double GimbalController::frictionFeedforward() const
{
  if (odom_ != nullptr)
  {
    return b_ * odom_->twist.twist.angular.z;
  }
  else
    return 0.;
}

void GimbalController::moveJoint(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  geometry_msgs::msg::Vector3 angular_vel_pitch, angular_vel_yaw;
  if (has_imu_)
  {
    geometry_msgs::msg::Vector3 gyro;
    gyro.x = imu_sensor_->get_angular_velocity()[0];
    gyro.y = imu_sensor_->get_angular_velocity()[1];
    gyro.z = imu_sensor_->get_angular_velocity()[2];
    try
    {
      tf2::doTransform(gyro, angular_vel_pitch,
                       tf_handler_->lookupTransform(joint_urdf_[0]->child_link_name, imu_name_, time));
      tf2::doTransform(gyro, angular_vel_yaw,
                       tf_handler_->lookupTransform(joint_urdf_[1]->child_link_name, imu_name_, time));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN_ONCE(get_node()->get_logger(), "%s", ex.what());
      return;
    }
  }
  else
  {
    angular_vel_pitch.y = joint_velocity_state_interface_[0].get().get_value();
    angular_vel_yaw.z = joint_velocity_state_interface_[1].get().get_value();
  }
  double roll_real, pitch_real, yaw_real, roll_des, pitch_des, yaw_des;
  quatToRPY(odom2gimbal_des_.transform.rotation, roll_des, pitch_des, yaw_des);
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);
  double pitch_angle_error = angles::shortest_angular_distance(pitch_real, pitch_des);
  double yaw_angle_error = angles::shortest_angular_distance(yaw_real, yaw_des);
  double pitch_vel_des = 0, yaw_vel_des = 0.;
  if (state_ == RATE)
  {
    if (cmd_gimbal_ != nullptr)
    {
      pitch_vel_des = cmd_gimbal_->rate_pitch;
      yaw_vel_des = cmd_gimbal_->rate_yaw;
    }
    tracking_differentiator_->update(yaw_des, yaw_vel_des);
  }
  else if (state_ == TRACK)
  {
    if (track_data_ != nullptr)
      bullet_solver_->getYawVelDes(yaw_vel_des);
    tracking_differentiator_->update(yaw_des, yaw_vel_des);
  }
  else if (state_ == TRAJ)
  {
    tracking_differentiator_->update(yaw_des);
    yaw_vel_des = tracking_differentiator_->getX2();
  }
  if (!pitch_des_in_limit_)
    pitch_vel_des = 0.;
  if (!yaw_des_in_limit_)
    yaw_vel_des = 0.;
  double yaw_angle_error_td = angles::shortest_angular_distance(yaw_real, tracking_differentiator_->getX1());
  double cmd_pitch = pid_pos_pitch_->computeCommand(pitch_angle_error, period);
  double cmd_yaw = pid_pos_yaw_->computeCommand(yaw_angle_error_td, period);
  cmd_pitch = pid_vel_pitch_->computeCommand(cmd_pitch + pitch_vel_des - angular_vel_pitch.y, period);
  cmd_yaw = pid_vel_yaw_->computeCommand(cmd_yaw + yaw_vel_des - angular_vel_yaw.z, period);
  joint_effort_command_interface_[0].get().set_value(cmd_pitch);
  joint_effort_command_interface_[1].get().set_value(cmd_yaw + frictionFeedforward());

  if (pitch_rt_pos_state_pub_)
  {
    if (pitch_rt_pos_state_pub_->trylock())
    {
      pitch_rt_pos_state_pub_->msg_.header.stamp = get_node()->get_clock()->now();
      pitch_rt_pos_state_pub_->msg_.error = pitch_angle_error;
      pitch_rt_pos_state_pub_->msg_.set_point = pitch_des;
      pitch_rt_pos_state_pub_->msg_.set_point_dot = pitch_vel_des;
      pitch_rt_pos_state_pub_->msg_.process_value = pitch_real;
      pitch_rt_pos_state_pub_->msg_.command = cmd_pitch;
      pitch_rt_pos_state_pub_->unlockAndPublish();
    }
  }
  if (yaw_rt_pos_state_pub_)
  {
    if (yaw_rt_pos_state_pub_->trylock())
    {
      yaw_rt_pos_state_pub_->msg_.header.stamp = get_node()->get_clock()->now();
      yaw_rt_pos_state_pub_->msg_.error = yaw_angle_error;
      yaw_rt_pos_state_pub_->msg_.set_point = yaw_des;
      yaw_rt_pos_state_pub_->msg_.set_point_dot = yaw_vel_des;
      yaw_rt_pos_state_pub_->msg_.td_set_point = tracking_differentiator_->getX1();
      yaw_rt_pos_state_pub_->msg_.td_set_point_dot = tracking_differentiator_->getX2();
      yaw_rt_pos_state_pub_->msg_.process_value = yaw_real;
      yaw_rt_pos_state_pub_->msg_.command = cmd_yaw;
      yaw_rt_pos_state_pub_->unlockAndPublish();
    }
  }
}

void GimbalController::setParameterEventCallBack()
{
  auto on_parameter_event_callback = [this](const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (auto& parameter : parameters)
    {
      const std::string& param_name = parameter.get_name();
      try
      {
        if (param_name == "b")
          b_ = parameter.get_value<double>();
        else if (param_name == "r")
          r_ = parameter.get_value<double>();
        else if (param_name == "h")
          h_ = parameter.get_value<double>();
      }
      catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
      {
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Please use the right type: " << e.what());
      }
    }
    return result;
  };
  parameter_callback_ =
      get_node()->get_node_parameters_interface()->add_on_set_parameters_callback(on_parameter_event_callback);
}
}  // namespace rm_ros2_gimbal_controller

PLUGINLIB_EXPORT_CLASS(rm_ros2_gimbal_controller::GimbalController, controller_interface::ControllerInterface)

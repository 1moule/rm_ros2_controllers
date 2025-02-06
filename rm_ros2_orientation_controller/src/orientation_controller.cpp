//
// Created by guanlin on 25-2-6.
//

#include "rm_ros2_orientation_controller/orientation_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace rm_ros2_orientation_controller
{
OrientationController::OrientationController() : ControllerInterface::ControllerInterface()
{
}

controller_interface::CallbackReturn OrientationController::on_init()
{
  // should have error handling
  imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
  frame_source_ = auto_declare<std::string>("frame_source", frame_source_);
  frame_target_ = auto_declare<std::string>("frame_target", frame_target_);
  timeout_ = auto_declare<double>("timeout", 0.1);
  imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", imu_interface_types_);

  tf_handler_ = std::make_shared<TfHandler>(get_node());
  tf_broadcaster_ = std::make_shared<TfRtBroadcaster>(get_node());

  last_imu_update_time_ = get_node()->get_clock()->now();

  source2target_msg_.header.frame_id = frame_source_;
  source2target_msg_.child_frame_id = frame_target_;
  source2target_msg_.transform.rotation.w = 1.0;

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OrientationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = { controller_interface::interface_configuration_type::INDIVIDUAL,
                                                        {} };
  return conf;
}

controller_interface::InterfaceConfiguration OrientationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = { controller_interface::interface_configuration_type::INDIVIDUAL,
                                                        {} };
  conf.names.reserve(imu_interface_types_.size());
  for (const auto& interface_type : imu_interface_types_)
  {
    conf.names.push_back(imu_name_ + "/" + interface_type);
  }
  return conf;
}

controller_interface::CallbackReturn OrientationController::on_configure(const rclcpp_lifecycle::State&)
{
  auto imuDataCallback = [this](const std::shared_ptr<sensor_msgs::msg::Imu> /*msg*/) -> void {
    last_imu_update_time_ = get_node()->get_clock()->now();
  };
  imu_data_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>("/" + imu_name_, rclcpp::SystemDefaultsQoS(),
                                                                         imuDataCallback);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OrientationController::on_activate(const rclcpp_lifecycle::State&)
{
  // clear out vectors in case of restart
  imu_state_interface_.clear();
  // assign state interfaces
  for (auto& interface : state_interfaces_)
  {
    if (interface.get_prefix_name() == imu_name_)
    {
      imu_state_interface_.emplace_back(interface);
    }
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type OrientationController::update(const rclcpp::Time& time,
                                                                const rclcpp::Duration& /*period*/)
{
  if ((time - last_imu_update_time_) < rclcpp::Duration::from_seconds(timeout_))
  {
    geometry_msgs::msg::TransformStamped source2target;
    source2target.header.stamp = time;
    source2target.header.stamp.nanosec += 1;  // Avoid redundant timestamp
    source2target_msg_.header.stamp = time;
    source2target_msg_.header.stamp.nanosec += 1;
    source2target_msg_ =
        getTransform(source2target, imu_state_interface_[1].get().get_value(),
                     imu_state_interface_[2].get().get_value(), imu_state_interface_[3].get().get_value(),
                     imu_state_interface_[0].get().get_value()) ?
            source2target :
            source2target_msg_;
    tf_handler_->setTransform(source2target_msg_, "rm_orientation_controller");
    tf_broadcaster_->sendTransform(source2target_msg_);
  }

  return controller_interface::return_type::OK;
}

bool OrientationController::getTransform(geometry_msgs::msg::TransformStamped& source2target, const double x,
                                         const double y, const double z, const double w)
{
  source2target.header.frame_id = frame_source_;
  source2target.child_frame_id = frame_target_;
  source2target.transform.rotation.w = 1.0;
  tf2::Transform source2odom, odom2fixed, fixed2target;
  try
  {
    geometry_msgs::msg::TransformStamped tf_msg = tf_handler_->lookupTransform(frame_source_, "odom");
    tf2::fromMsg(tf_msg.transform, source2odom);
    tf_msg = tf_handler_->lookupTransform("odom", imu_name_);
    tf2::fromMsg(tf_msg.transform, odom2fixed);
    tf_msg = tf_handler_->lookupTransform(imu_name_, frame_target_);
    tf2::fromMsg(tf_msg.transform, fixed2target);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(get_node()->get_logger(), "%s", ex.what());
    return false;
  }
  tf2::Quaternion odom2fixed_quat;
  odom2fixed_quat.setValue(x, y, z, w);
  odom2fixed.setRotation(odom2fixed_quat);
  source2target.transform = tf2::toMsg(source2odom * odom2fixed * fixed2target);
  return true;
}
}  // namespace rm_ros2_orientation_controller

PLUGINLIB_EXPORT_CLASS(rm_ros2_orientation_controller::OrientationController, controller_interface::ControllerInterface)

//
// Created by guanlin on 25-2-5.
//
#include <robot_state_controller/robot_state_controller.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

namespace robot_state_controller
{
RobotStateController::RobotStateController() : ControllerInterface::ControllerInterface()
{
}

controller_interface::CallbackReturn RobotStateController::on_init()
{
  // should have error handling
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
  publish_rate_ = auto_declare<double>("publish_rate", 50.0);
  use_tf_static_ = auto_declare<bool>("use_tf_static", true);

  tf_handler_ = std::make_shared<TfHandler>(get_node());
  tf_broadcaster_ = std::make_shared<TfRtBroadcaster>(get_node());
  static_tf_broadcaster_ = std::make_shared<StaticTfRtBroadcaster>(get_node());

  last_update_ = get_node()->get_clock()->now();
  last_publish_time_ = get_node()->get_clock()->now();

  std::string urdf_string;
  get_node()->get_parameter_or<std::string>("robot_description", urdf_string, "");
  if (!model_.initString(urdf_string))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to init URDF from robot description");
    return CallbackReturn::ERROR;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model_, tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to extract kdl tree from xml robot description");
    return CallbackReturn::ERROR;
  }
  addChildren(tree.getRootSegment());
  for (auto& item : segments_)
  {
    if (item.first.find("roller") == std::string::npos)
    {
      joint_names_.push_back(item.first);
    }
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotStateController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration RobotStateController::state_interface_configuration() const
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

controller_interface::CallbackReturn RobotStateController::on_configure(const rclcpp_lifecycle::State&)
{
  auto tfSubCallback = [this](const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) -> void {
    tf_msg_.writeFromNonRT(msg);
  };
  tf_sub_ =
      get_node()->create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS(), tfSubCallback);
  auto staticSubCallback = [this](const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) -> void {
    tf_static_msg_.writeFromNonRT(msg);
  };
  tf_static_sub_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::SystemDefaultsQoS(),
                                                                             staticSubCallback);

  // state_pub_ = get_node()->create_publisher<rm_ros2_msgs::msg::GimbalPosState>(
  //     std::string(get_node()->get_name()) + "/pos_state", rclcpp::SystemDefaultsQoS());
  // rt_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::GimbalPosState>>(state_pub_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotStateController::on_activate(const rclcpp_lifecycle::State&)
{
  // clear out vectors in case of restart
  // joint_effort_command_interface_.clear();
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
    if (interface.get_interface_name() == "position")
    {
      if (interface.get_name().find("roller") == std::string::npos)
        jnt_states_.insert(std::make_pair<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
            interface.get_name().c_str(), interface));
    }
  }
  return CallbackReturn::SUCCESS;
}

std::string stripSlash(const std::string& in)
{
  if (!in.empty() && in[0] == '/')
  {
    return in.substr(1);
  }
  return in;
}

controller_interface::return_type RobotStateController::update(const rclcpp::Time& time,
                                                               const rclcpp::Duration& /*period*/)
{
  if (last_update_ > time)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "Moved backwards in time (probably because ROS clock was reset), clear all tf buffer!");
    tf_handler_->clear();
  }
  last_update_ = time;
  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms, tf_static_transforms;
  geometry_msgs::msg::TransformStamped tf_transform;
  // Loop over all float segments
  for (auto& item : segments_)
  {
    auto jnt_iter = jnt_states_.find(item.first + "/position");
    if (jnt_iter != jnt_states_.end())
      tf_transform = tf2::kdlToTransform(item.second.segment.pose(jnt_iter->second.get().get_value()));
    else
      continue;
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = stripSlash(item.second.root);
    tf_transform.child_frame_id = stripSlash(item.second.tip);
    tf_transforms.push_back(tf_transform);
  }
  // Loop over all fixed segments
  for (std::map<std::string, SegmentPair>::const_iterator seg = segments_fixed_.begin(); seg != segments_fixed_.end();
       seg++)
  {
    tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = stripSlash(seg->second.root);
    tf_transform.child_frame_id = stripSlash(seg->second.tip);
    tf_static_transforms.push_back(tf_transform);
  }
  for (const auto& tran : tf_transforms)
    tf_handler_->setTransform(tran, "robot_state_controller", false);
  for (const auto& tran : tf_static_transforms)
    tf_handler_->setTransform(tran, "robot_state_controller", true);
  if (publish_rate_ > 0.0 && last_publish_time_ + rclcpp::Duration::from_seconds(1.0 / publish_rate_) <= time)
  {
    tf_broadcaster_->sendTransform(tf_transforms);
    if (use_tf_static_)
      static_tf_broadcaster_->sendTransform(tf_static_transforms);
    else
      tf_broadcaster_->sendTransform(tf_static_transforms);
    last_publish_time_ = time;
  }
  tf_transforms.clear();
  tf_static_transforms.clear();
  // // Loop over subscribe
  auto tf = *tf_msg_.readFromRT();
  if (tf != nullptr)
  {
    for (const auto& item : tf.get()->transforms)
    {
      try
      {
        if (item.header.stamp !=
            tf_handler_->lookupTransform(item.child_frame_id, item.header.frame_id, item.header.stamp).header.stamp)
          tf_transforms.push_back(item);
      }
      catch (tf2::TransformException& ex)
      {
        tf_transforms.push_back(item);
      }
    }
  }
  auto tf_static = *tf_static_msg_.readFromRT();
  if (tf_static != nullptr)
  {
    for (const auto& item : tf_static.get()->transforms)
    {
      try
      {
        if (item.header.stamp !=
            tf_handler_->lookupTransform(item.child_frame_id, item.header.frame_id, item.header.stamp).header.stamp)
          tf_static_transforms.push_back(item);
      }
      catch (tf2::TransformException& ex)
      {
        tf_static_transforms.push_back(item);
      }
    }
  }

  // tf_msg_.readFromRT()->get()->transforms.clear();
  // tf_static_msg_.readFromRT()->get()->transforms.clear();
  for (const auto& tran : tf_transforms)
    tf_handler_->setTransform(tran, "outside", false);
  for (const auto& tran : tf_static_transforms)
    tf_handler_->setTransform(tran, "outside", true);

  return controller_interface::return_type::OK;
}

void RobotStateController::addChildren(KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (auto i : children)
  {
    const KDL::Segment& child = GetTreeElementSegment(i->second);
    SegmentPair s(GetTreeElementSegment(i->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None)
    {
      if (model_.getJoint(child.getJoint().getName()) &&
          model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
      {
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info",
            root.c_str(), child.getName().c_str());
      }
      else
      {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        RCLCPP_INFO(get_node()->get_logger(), "Adding fixed segment from %s to %s", root.c_str(),
                    child.getName().c_str());
      }
    }
    else
    {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      RCLCPP_INFO(get_node()->get_logger(), "Adding moving segment from %s to %s", root.c_str(),
                  child.getName().c_str());
    }
    addChildren(i);
  }
}

}  // namespace robot_state_controller

PLUGINLIB_EXPORT_CLASS(robot_state_controller::RobotStateController, controller_interface::ControllerInterface)

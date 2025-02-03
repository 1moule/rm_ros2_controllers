//
// Created by guanlin on 24-12-25.
//

#include "rm_ros2_chassis_controllers/chassis_base.hpp"

namespace rm_ros2_chassis_controllers {
ChassisBase::ChassisBase(): controller_interface::ControllerInterface() {}
controller_interface::CallbackReturn ChassisBase::on_init()
{
    // should have error handling
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    publish_rate_=auto_declare<double>("publish_rate", 100.0);

    cmd_chassis_=std::shared_ptr<rm_ros2_msgs::msg::ChassisCmd>();
    cmd_vel_=std::make_shared<geometry_msgs::msg::Twist>();
    vel_base_=std::make_shared<geometry_msgs::msg::Twist>();
    vel_cmd_=std::make_shared<geometry_msgs::msg::Vector3>();

    last_publish_time_ = get_node()->get_clock()->now();

    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChassisBase::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto & joint_name : joint_names_)
    {
        for (const auto & interface_type : command_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

controller_interface::InterfaceConfiguration ChassisBase::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto & joint_name : joint_names_)
    {
        for (const auto & interface_type : state_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

controller_interface::CallbackReturn ChassisBase::on_configure(const rclcpp_lifecycle::State &)
{
    // cmd chassis
    auto cmdChassisCallback =
      [this](const std::shared_ptr<rm_ros2_msgs::msg::ChassisCmd> msg) -> void
      {
          cmd_chassis_buffer_.writeFromNonRT(msg);
      };
    cmd_chassis_sub_ =
      get_node()->create_subscription<rm_ros2_msgs::msg::ChassisCmd>(
        "/cmd_chassis", rclcpp::SystemDefaultsQoS(), cmdChassisCallback);

    // cmd vel
    auto cmdVelCallback =
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg)->void {
            cmd_vel_buffer_.writeFromNonRT(msg);
            stamp_=rclcpp::Clock().now();
        };
    cmd_vel_sub_ =get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),cmdVelCallback);

    odom_pub_nonrt_=get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());
    odom_pub_=std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_nonrt_);
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = "base_link";
    odom_pub_->msg_.twist.covariance = { 0.01, 0., 0., 0., 0., 0., 0.,
                                            0.01, 0., 0., 0., 0., 0., 0.,
                                            0.01, 0., 0., 0., 0., 0., 0.,
                                            0.01, 0., 0., 0., 0., 0., 0.,
                                            0.01, 0., 0., 0., 0., 0., 0.,
                                            0.01 };

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisBase::on_activate(const rclcpp_lifecycle::State &)
{
    // clear out vectors in case of restart
    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_effort_state_interface_.clear();
    // assign command interfaces
    for (auto & interface : command_interfaces_)
    {
        command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }
    // assign state interfaces
    for (auto & interface : state_interfaces_)
    {
        state_interface_map_[interface.get_interface_name()]->push_back(interface);
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type ChassisBase::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    cmd_chassis_=*cmd_chassis_buffer_.readFromRT();
    cmd_vel_=*cmd_vel_buffer_.readFromRT();
    if (cmd_vel_!=nullptr) {
        vel_cmd_->x=cmd_vel_->linear.x;
        vel_cmd_->y=cmd_vel_->linear.y;
        vel_cmd_->z=cmd_vel_->angular.z;
    }
    moveJoint();
    updateOdom(time);
    return controller_interface::return_type::OK;
}

void ChassisBase::updateOdom(const rclcpp::Time& time){
    odometry();
    if (publish_rate_ > 0.0 && last_publish_time_+rclcpp::Duration::from_seconds(1.0 / publish_rate_) < time)
    {
        if (odom_pub_->trylock())
        {
          odom_pub_->msg_.header.stamp = time;
          odom_pub_->msg_.twist.twist.linear.x = vel_base_->linear.x;
          odom_pub_->msg_.twist.twist.linear.y = vel_base_->linear.y;
          odom_pub_->msg_.twist.twist.angular.z = vel_base_->angular.z;
          odom_pub_->unlockAndPublish();
        }
        last_publish_time_ = time;
    }
}

}
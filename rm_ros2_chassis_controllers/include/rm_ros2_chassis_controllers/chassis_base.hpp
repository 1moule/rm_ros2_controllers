/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by huakang on 2021/3/21.
//

#pragma once

#include <controller_interface/controller_interface.hpp>
//#include <hardware_interface/joint_command_interface.h>
//#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/control_toolbox/pid.hpp>
//#include <rm_common/filters/filters.h>
//#include <effort_controllers/joint_velocity_controller.h>
#include <rm_ros2_msgs/msg/chassis_cmd.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rm_chassis_controllers
{
struct Command
{
    cmd_vel_=geometry_msgs::msg::Twist();
    rm_ros2_msgs::msg::ChassisCmd cmd_chassis_;
    rclcpp::Time stamp_;
};
class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);  // CHANGE
        timer_ = this->create_wall_timer(
                500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = tutorial_interfaces::msg::Num();                                   // CHANGE
        message.num = this->count_++;                                                     // CHANGE
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");    // CHANGE
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;             // CHANGE
    size_t count_;
};
template <typename... T>
class ChassisBase : public controller_interface::ControllerInterface
{
public:
    CONTROLLER_INTERFACE_PUBLIC
    ChassisBase() = default;
    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

protected:
    /** @briefThe The mode RAW: original state.
     *
     *  The mode raw: original state. Linear velocity will be set zero to stop move.
     */
    void raw();
    /** @brief The mode FOLLOW: chassis will follow gimbal.
     *
     * The mode FOLLOW: The chassis's direct will follow gimbal's direct all the time.
     *
     * @param time The current time.
     * @param period The time passed since the last call to update.
     */
    void follow(const rclcpp::Time& time, const rclcpp::Duration& period);
    /** @brief The mode TWIST: Just moving chassis.
     *
     * The mode TWIST: Chassis will move independent and will not effect by gimbal's move.
     *
     * @param time The current time.
     * @param period The time passed since the last call to update.
     */
    void twist(const rclcpp::Time& time, const rclcpp::Duration& period);
    virtual void moveJoint(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
    virtual geometry_msgs::msg::Twist odometry() = 0;
    /** @brief Init frame on base_link. Integral vel to pos and angle.
     *
     * @param time The current time.
     * @param period The time passed since the last call to update.
     */
    void updateOdom(const rclcpp::Time& time, const rclcpp::Duration& period);
    /** @brief Set chassis velocity to zero.
     */
    void recovery();
    /** @brief Transform tf velocity to base link frame.
     *
     * @param from The father frame.
     */
    void tfVelToBase(const std::string& from);
    /** @brief To limit the chassis power according to current power limit.
     *
     * Receive power limit from command. Set max_effort command to chassis to avoid exceed power limit.
     */
    void powerLimit();
    /** @brief Write current command from rm_msgs::ChassisCmd.
     *
     * @param msg This message contains various state parameter settings for basic chassis control
     */
    void cmdChassisCallback(const rm_ros2_msgs::msg::ChassisCmd::SharedPtr msg);
    /** @brief Write current command from  geometry_msgs::Twist.
     *
     * @param msg This expresses velocity in free space broken into its linear and angular parts.
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void outsideOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rm_control::RobotStateHandle robot_state_handle_{};
    hardware_interface::EffortJointInterface* effort_joint_interface_{};
    std::vector<hardware_interface::JointHandle> joint_handles_{};

    double wheel_radius_{}, publish_rate_{}, twist_angular_{}, timeout_{}, effort_coeff_{}, velocity_coeff_{},
            power_offset_{};
    double max_odom_vel_;
    bool enable_odom_tf_ = false;
    bool topic_update_ = false;
    bool publish_odom_tf_ = false;
    bool state_changed_ = true;
    enum
    {
        RAW,
        FOLLOW,
        TWIST
    };
    int state_ = RAW;
    RampFilter<double>*ramp_x_{}, *ramp_y_{}, *ramp_w_{};
    std::string follow_source_frame_{}, command_source_frame_{};

    rclcpp::Time last_publish_time_;
    geometry_msgs::msg::TransformStamped odom2base_{};
    tf2::Transform world2odom_;
    geometry_msgs::msg::Vector3 vel_cmd_{};  // x, y
    control_toolbox::Pid pid_follow_;

    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::> > odom_pub_;
    rm_common::TfRtBroadcaster tf_broadcaster_{};
//    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr outside_odom_sub_;
//    rclcpp::Subscription<rm_ros2_msgs::msg::ChassisCmd>::SharedPtr cmd_chassis_sub_;
//    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    Command cmd_struct_;
    realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
    realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> odom_buffer_;
};

}  // namespace rm_chassis_controllers

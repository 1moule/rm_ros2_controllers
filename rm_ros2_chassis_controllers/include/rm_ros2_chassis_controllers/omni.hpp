//
// Created by guanlin on 25-2-2.
//

#ifndef OMNI_HPP
#define OMNI_HPP

#include "rm_ros2_chassis_controllers/chassis_base.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>

namespace rm_ros2_chassis_controllers
{
class OmniController : public ChassisBase
{
public:
    OmniController();
    controller_interface::CallbackReturn on_init() override;
private:
    // void moveJoint(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    // geometry_msgs::msg::Twist odometry() override;

    // Eigen::MatrixXd chassis2joints_;
    //
    // std::vector<std::array<double,3>> wheels_pos_;
    // std::vector<double> roller_angles_;
    // double radius_;
};
}

#endif //OMNI_HPP

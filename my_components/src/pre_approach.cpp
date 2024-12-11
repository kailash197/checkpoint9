#include "my_components/pre_approach.hpp"

#include "helper_package/helper_functions.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace helper_package;

namespace my_components {
PreApproach::PreApproach(const rclcpp::NodeOptions &options): Node("pre_approach_node", options) {
  obstacle = OBSTACLE;
  degrees = normalize_angle(deg2rad(DEGREES));
  RCLCPP_INFO(this->get_logger(), "Obstacle parameter: %f", OBSTACLE);
  RCLCPP_INFO(this->get_logger(), "Degrees parameter: %d", DEGREES);

  timer_cb_group_ = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
    laser_cb_group_ = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
    odom_cb_group_ = this->create_callback_group(CallbackGroupType::MutuallyExclusive);

    auto laser_sub_options = SubscriptionOptions(); laser_sub_options.callback_group = laser_cb_group_;
    auto odom_sub_options = SubscriptionOptions(); odom_sub_options.callback_group = odom_cb_group_;

    laser_sub_ = this->create_subscription<LaserScan>(
        SCAN_TOPIC, QoS(10).best_effort(),
        std::bind(&PreApproach::laser_scan_callback, this, std::placeholders::_1),
        laser_sub_options);
    odom_sub_ = this->create_subscription<Odometry>(
        ODOM_TOPIC, QoS(10).reliable(),
        std::bind(&PreApproach::odom_callback, this,std::placeholders::_1),
        odom_sub_options);      
    cmd_pub_ = this->create_publisher<Twist>(CMD_TOPIC, QoS(10).best_effort());
    timer_ = this->create_wall_timer(50ms, std::bind(&PreApproach::timer_callback, this), timer_cb_group_);
    RCLCPP_INFO(this->get_logger(), "Preapproach Node Ready.");
}

void PreApproach::odom_callback(const Odometry::SharedPtr odom_msg) {
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
    current_yaw_ = normalize_angle(tf2::getYaw(q));
}

void PreApproach::laser_scan_callback(const LaserScan::SharedPtr scan_msg){
    distance_forward_ = 0.0;
    if(first_run){
        //find indices of scan msg corresponding to forward direction
        index_forward = find_index_scan_msg(scan_msg);
        first_run = false;
    }
    for (int i = index_forward-5; i < index_forward+5; i++){
        distance_forward_ += scan_msg->ranges[i];
    }
    distance_forward_ /= 10.0;
}

void PreApproach::timer_callback() {
    if (!reached_obstacle){
        if (distance_forward_ - obstacle > 0.5) {
            publish_velocity(LINEAR_VEL, 0.0);
        } else if (distance_forward_ - obstacle > 0.01) {
            publish_velocity(LINEAR_VEL/3.0, 0.0);
        } else {
            publish_velocity(0.0, 0.0);
            reached_obstacle = true;
            desired_yaw_ = normalize_angle((current_yaw_+ degrees));
            RCLCPP_DEBUG(this->get_logger(), "Distance: %.2f", distance_forward_);
        }
        
    } else {
        if (fabs(desired_yaw_ - current_yaw_) > 0.05){
            publish_velocity(0.0, 2*(desired_yaw_ - current_yaw_));      
        } else if (fabs(desired_yaw_ - current_yaw_) > 0.005){
            publish_velocity(0.0, 0.1);
        } else {
            // end of node execution
            RCLCPP_INFO(this->get_logger(), "End of Preapproach.");
            RCLCPP_DEBUG(this->get_logger(), "Current yaw: %.2f, desired: %.2f",current_yaw_, desired_yaw_);
            publish_velocity(0.0, 0.0);
            // rclcpp::shutdown();
            timer_->cancel();

        }
    }
}

void PreApproach::publish_velocity(double linear, double angular) {
    auto cmd_msg = Twist();
    cmd_msg.linear.x = linear;
    cmd_msg.angular.z = angular;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
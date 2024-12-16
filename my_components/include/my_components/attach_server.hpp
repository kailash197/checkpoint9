#ifndef COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_
#define COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "my_components/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"

using namespace rclcpp;
using namespace tf2_ros;

using GoToLoading = my_components::srv::GoToLoading;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using LaserScan = sensor_msgs::msg::LaserScan;
using Groups = std::vector<std::tuple<size_t, size_t, std::pair<double, double>>>;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Pose2D = geometry_msgs::msg::Pose2D;

namespace my_components {
    
    class AttachServer : public Node {
        private:
        // Features
        Service<GoToLoading>::SharedPtr server_;
        Subscription<LaserScan>::SharedPtr laser_sub_;
        Subscription<Odometry>::SharedPtr odom_sub_;
        Publisher<Twist>::SharedPtr cmd_vel_pub_;
        Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;
        std::unique_ptr<TransformBroadcaster> cart_tf_broadcaster_;

        void service_callback(
            const std::shared_ptr<GoToLoading::Request> request,
            const std::shared_ptr<GoToLoading::Response> response);
        void laser_scan_callback(const LaserScan::SharedPtr scan_msg);
        void odom_callback(const Odometry::SharedPtr odom_msg);

        CallbackGroup::SharedPtr laser_cb_group_;
        CallbackGroup::SharedPtr odom_cb_group_;

        TransformStamped cart_transform_;
        Buffer tf_buffer_;
        TransformListener tf_listener_;
        Pose2D current_pos_;
        
        bool found_both_legs_=false;

        public:
            explicit AttachServer(const rclcpp::NodeOptions & options);
            void publish_velocity(double linear, double angular);
            Groups find_midpoint_intensity_groups(const LaserScan::SharedPtr scan_msg,float threshold);

    };

} // namespace my_components

#endif // COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_

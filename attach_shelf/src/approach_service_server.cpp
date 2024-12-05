#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include <algorithm> //std::find_if()
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace tf2_ros;

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using GoToLoading = attach_shelf::srv::GoToLoading;
using Groups = std::vector<std::tuple<size_t, size_t, std::pair<double, double>>>;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Pose2D = geometry_msgs::msg::Pose2D;

const string SCAN_TOPIC = "/scan";
const string CMD_TOPIC = "/diffbot_base_controller/cmd_vel_unstamped";
const string SERVICE_NAME = "/approach_shelf";
constexpr double LINEAR_VEL = 0.80;
constexpr float INTENSITY_THRESHOLD = 7000;
const string TARGET_FRAME = "robot_front_laser_base_link";
const string SOURCE_FRAME = "cart_frame";
const string REFERENCE_FRAME = "odom";
const string ODOM_TOPIC = "/diffbot_base_controller/odom";
constexpr double DISTANCE_FORWARD = 0.30; //meters

// Function to normalize angle to [-PI, PI]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

class AppoarchServiceServerNode : public Node {
    private:
        Service<GoToLoading>::SharedPtr server_;
        void service_callback(
            const std::shared_ptr<GoToLoading::Request> request,
            const std::shared_ptr<GoToLoading::Response> response);
        Subscription<LaserScan>::SharedPtr laser_sub_;
        CallbackGroup::SharedPtr laser_cb_group_;
        void laser_scan_callback(const LaserScan::SharedPtr scan_msg);
        Groups find_midpoint_intensity_groups(const LaserScan::SharedPtr scan_msg,float threshold);
        std::unique_ptr<TransformBroadcaster> cart_tf_broadcaster_;
        TransformStamped cart_transform_;
        Buffer tf_buffer_;
        TransformListener tf_listener_;
        bool found_both_legs_=false;
        Publisher<Twist>::SharedPtr cmd_vel_pub_;
        Subscription<Odometry>::SharedPtr odom_sub_;
        void odom_callback(const Odometry::SharedPtr odom_msg);
        CallbackGroup::SharedPtr odom_cb_group_;
        Pose2D current_pos_;
    public:
        void publish_velocity(double linear, double angular);
        AppoarchServiceServerNode():Node("approach_shelf_server_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_){
            server_ = this->create_service<GoToLoading>(
                SERVICE_NAME,
                std::bind(&AppoarchServiceServerNode::service_callback, this, _1, _2));
            laser_cb_group_ = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
            auto laser_sub_options = SubscriptionOptions(); laser_sub_options.callback_group = laser_cb_group_;
            laser_sub_ = this->create_subscription<LaserScan>(
                SCAN_TOPIC, QoS(10).best_effort(),
                std::bind(&AppoarchServiceServerNode::laser_scan_callback, this, _1),
                laser_sub_options);
            cart_tf_broadcaster_ = std::make_unique<TransformBroadcaster>(*this);
            cart_transform_.header.frame_id = TARGET_FRAME;
            cart_transform_.child_frame_id = SOURCE_FRAME;
            cmd_vel_pub_ = this->create_publisher<Twist>(CMD_TOPIC, QoS(10).best_effort());
            odom_cb_group_ = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
            auto odom_sub_options = SubscriptionOptions(); odom_sub_options.callback_group = odom_cb_group_;
            odom_sub_ = this->create_subscription<Odometry>(
                ODOM_TOPIC, QoS(10).reliable(),
                std::bind(&AppoarchServiceServerNode::odom_callback, this,_1),
                odom_sub_options);
            RCLCPP_INFO(this->get_logger(), "Service Server Ready.");
        }
};

void AppoarchServiceServerNode::odom_callback(const Odometry::SharedPtr odom_msg) {
    RCLCPP_DEBUG(get_logger(), "Inside ODOM Callback");
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);

    current_pos_.x = odom_msg->pose.pose.position.x;
    current_pos_.y = odom_msg->pose.pose.position.y;
    current_pos_.theta = normalize_angle(tf2::getYaw(q));
}

void AppoarchServiceServerNode::service_callback(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {
        /*
        detect the legs of the shelf using the laser intensity values
            - if the laser only detects 1 shelf leg or none: Return False
            - If it detects both legs, the service will publish a transform named cart_frame to the center point between both legs.
        */
        RCLCPP_INFO(this->get_logger(), "Service Request Received.");
        bool attach_to_shelf = request->attach_to_shelf;
        //i. publish cart_frame transform, in both cases
        if (found_both_legs_){
            cart_tf_broadcaster_->sendTransform(cart_transform_);
        } else {
            // False: if the laser only detects 1 shelf leg or none
            response->complete = false;
            RCLCPP_INFO(this->get_logger(), "Service Completed.");
            return;
        }
        if (attach_to_shelf){
            RCLCPP_INFO(this->get_logger(), "State: Attach to shelf.");
            //perform final approach
            //i. move robot underneathe the shelf
            // Calculate distance
            double dx = cart_transform_.transform.translation.x;
            double dy = cart_transform_.transform.translation.y;
            double dz = cart_transform_.transform.translation.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // Calculate angle (rotation about Z-axis)
            double yaw = atan2(dy, dx);

            Twist cmd_vel_;

            while (rclcpp::ok() && distance > 0.15){
                publish_velocity(0.30, 0.4 * yaw);

                dx = cart_transform_.transform.translation.x;
                dy = cart_transform_.transform.translation.y;
                dz = cart_transform_.transform.translation.z;
                distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                yaw = atan2(dy, dx);
                RCLCPP_DEBUG(get_logger(), "Distance to cart TF: %.4fm", distance);
            }
            publish_velocity(0.0, 0.0);

            //move 30cm further
            RCLCPP_INFO(this->get_logger(), "State: Move forward 30cm.");
            rclcpp::sleep_for(100ms);
            yaw = current_pos_.theta;
            double x_target = current_pos_.x + DISTANCE_FORWARD * cos(yaw);
            double y_target = current_pos_.y + DISTANCE_FORWARD * sin(yaw);
            dx = x_target;
            dy = y_target;
            distance = std::sqrt(dx * dx + dy * dy);

            while (distance > 0.01){
                publish_velocity(0.10, 0.0);
                //update
                yaw = current_pos_.theta;
                dx = x_target - current_pos_.x;
                dy = y_target - current_pos_.y;
                distance = std::sqrt(dx * dx + dy * dy);
                RCLCPP_INFO(this->get_logger(), "Distance: %.2f.", distance);
            }
            publish_velocity(0.0, 0.0);

            //ii. lift the shelf
            RCLCPP_INFO(this->get_logger(), "State: Lift the shelf.");

        }

        // True: only if the final approach is successful
        response->complete = true;
        RCLCPP_INFO(this->get_logger(), "Service Completed.");
}

void AppoarchServiceServerNode::laser_scan_callback(const LaserScan::SharedPtr scan_msg){
    RCLCPP_DEBUG(get_logger(), "Inside LASER Callback");
    auto groups = find_midpoint_intensity_groups(scan_msg, INTENSITY_THRESHOLD);
    found_both_legs_ = (groups.size() == 2 );
    if (found_both_legs_){
        double cart_x = (std::get<2>(groups[0]).first + std::get<2>(groups[1]).first)/2.0;
        double cart_y = (std::get<2>(groups[0]).second + std::get<2>(groups[1]).second)/2.0;
        RCLCPP_DEBUG(this->get_logger(), "Cart position: (%.2f, %.2f)", cart_x, cart_y);
        /*
        fix Waiting for transform odom ->  cart_frame
        Lookup would require extrapolation into the past
        */
        cart_transform_.header.stamp = this->get_clock()->now();
        TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(
                REFERENCE_FRAME,
                TARGET_FRAME,
                tf2::TimePointZero);
            cart_transform_.header.stamp = transform.header.stamp;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
        cart_transform_.transform.translation.x = cart_x;
        cart_transform_.transform.translation.y = cart_y;
        cart_transform_.transform.translation.z = 0.0;
        cart_transform_.transform.rotation.x = 0.0;
        cart_transform_.transform.rotation.y = 0.0;
        cart_transform_.transform.rotation.z = 0.0;
        cart_transform_.transform.rotation.w = 1.0;
    }        
}

Groups AppoarchServiceServerNode::find_midpoint_intensity_groups(const LaserScan::SharedPtr scan_msg,float threshold ) {
    Groups groups;
    auto it = scan_msg->intensities.begin();
    while (it != scan_msg->intensities.end()) {
        it = std::find_if(it, scan_msg->intensities.end(), [threshold](float intensity) {
            return intensity > threshold;
        });
        if (it != scan_msg->intensities.end()){
            size_t start_ind = std::distance(scan_msg->intensities.begin(), it);
            it = std::find_if(it, scan_msg->intensities.end(), [threshold](float intensity) {
                return intensity <= threshold;
            });
            size_t end_ind = std::distance(scan_msg->intensities.begin(), it) - 1;

            // compute midpoint
            double sum_x = 0.0, sum_y = 0.0;
            size_t count = 0;
            double angle;
            for (size_t i = start_ind; i <= end_ind; i++){
                if (scan_msg->ranges[i] > 0.0){
                    angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                    sum_x += scan_msg->ranges[i] * std::cos(angle);
                    sum_y += scan_msg->ranges[i] * std::sin(angle);
                    count++;
                }
            }
            std::pair<double, double> midpoint = {sum_x / count, sum_y / count};
            groups.emplace_back(start_ind, end_ind, midpoint);
        }
    }
    return groups;
}

void AppoarchServiceServerNode::publish_velocity(double linear, double angular) {
    auto cmd_msg = Twist();
    cmd_msg.linear.x = linear;
    cmd_msg.angular.z = angular;
    cmd_vel_pub_->publish(cmd_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AppoarchServiceServerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

/*
Service: /approach_shelf
The service will do the following:
It will detect the legs of the shelf using the laser intensity values (check the Laser Intensities section below)
If the laser only detects 1 shelf leg or none, it will return a False message.
If it detects both legs, the service will publish a transform named cart_frame to the center point between both legs.
Then, the robot will use this TF to move towards the shelf (using the transform coordinates).
Once the robot has reached the TF coordinates, it will move forward 30 cm more (to end up right underneath the shelf).
*/

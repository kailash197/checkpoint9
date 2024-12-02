#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include <algorithm> //std::find_if()
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace tf2_ros;

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
// using Odometry = nav_msgs::msg::Odometry;
using GoToLoading = attach_shelf::srv::GoToLoading;
using Groups = std::vector<std::tuple<size_t, size_t, std::pair<double, double>>>;
using TransformStamped = geometry_msgs::msg::TransformStamped;

const string SCAN_TOPIC = "/scan";
const string CMD_TOPIC = "/diffbot_base_controller/cmd_vel_unstamped";
const string SERVICE_NAME = "/approach_shelf";
constexpr double LINEAR_VEL = 0.80;
constexpr float INTENSITY_THRESHOLD = 7000;
const string TARGET_FRAME = "robot_front_laser_base_link";
const string SOURCE_FRAME = "cart_frame";
const string REFERENCE_FRAME = "odom";

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
    public:
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
            RCLCPP_INFO(this->get_logger(), "Service Server Ready.");
        }
};

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
            RCLCPP_INFO(get_logger(), "Service Completed.");
            return;
        }
        if (attach_to_shelf){
            //perform final approach
            //i. move robot underneathe the shelf
            // Calculate distance
            double dx = cart_transform_.transform.translation.x;
            double dy = cart_transform_.transform.translation.y;
            double dz = cart_transform_.transform.translation.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // Calculate angle (rotation about Z-axis)
            double yaw = atan2(dy, dx);

            // assign velocities
            double kp_distance = 1.0;
            double kp_yaw = 1.0;
            double linear_vel = 0.0;
            double angular_vel = 0.0;

            Twist cmd_vel_;

            while (distance > 0.01){
                if(distance > 1.0 ){
                    kp_distance = 1.0;
                    kp_yaw = 1.0;
                } else {
                    kp_distance = 0.5;
                    kp_yaw=0.5;
                }
                if (fabs(yaw) > M_PI / 6.0){
                    kp_distance *= 0.5;
                    kp_yaw *= 1.2;
                }

                linear_vel = kp_distance * distance;
                angular_vel = kp_yaw * yaw;
                cmd_vel_.linear.x = linear_vel;
                cmd_vel_.angular.z = angular_vel;
                this->cmd_vel_pub_->publish(cmd_vel_);
            }
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            this->cmd_vel_pub_->publish(cmd_vel_);

            //move 30cm further
            //ii. lift the shelf

        }

        // True: only if the final approach is successful
        response->complete = true;
        RCLCPP_INFO(this->get_logger(), "Service Completed.");
}

void AppoarchServiceServerNode::laser_scan_callback(const LaserScan::SharedPtr scan_msg){
    auto groups = find_midpoint_intensity_groups(scan_msg, INTENSITY_THRESHOLD);
    found_both_legs_ = (groups.size() == 2 );
    if (found_both_legs_){
        double cart_x = (std::get<2>(groups[0]).first + std::get<2>(groups[1]).first)/2.0;
        double cart_y = (std::get<2>(groups[0]).second + std::get<2>(groups[1]).second)/2.0;
        RCLCPP_INFO(this->get_logger(), "Cart position: (%.2f, %.2f)", cart_x, cart_y);
        /*
        fix Waiting for transform odom ->  cart_frame
        Lookup would require extrapolation into the past
        */
        TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(
                REFERENCE_FRAME,
                TARGET_FRAME,
                tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }
        cart_transform_.header.stamp = transform.header.stamp;
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AppoarchServiceServerNode>());
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
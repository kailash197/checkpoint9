#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <tuple>
#include <sstream>   // For std::ostringstream
#include <stdexcept>

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using Pose2D = geometry_msgs::msg::Pose2D;

const string SCAN_TOPIC = "/scan";
const string ODOM_TOPIC = "/diffbot_base_controller/odom";
const string CMD_TOPIC = "/diffbot_base_controller/cmd_vel_unstamped";
constexpr double LINEAR_VEL = 0.50;

/* HELPERS */
// Function to find index for a given angle and specific points
int find_index_scan_msg(const LaserScan::SharedPtr scan_msg, double angle = 0.0) {
    if (!scan_msg) {
        RCLCPP_ERROR(rclcpp::get_logger("find_index_scan_msg"), "Invalid LaserScan message.");
        return -1;
    }

    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
        std::ostringstream error_msg;
        error_msg << "Angle out of bounds, [" << scan_msg->angle_min << " ," << scan_msg->angle_max << "]";
        throw std::invalid_argument(error_msg.str());
    }

    auto angle_to_index = [&](double ang) -> int {
        return static_cast<int>((ang - scan_msg->angle_min) / scan_msg->angle_increment);
    };

    int temp = angle_to_index(angle);
    RCLCPP_DEBUG(rclcpp::get_logger("find_index_scan_msg"), "Angle: %f, Index: %d", angle, temp);
    return temp;
}

// Function to normalize angle to [-PI, PI]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Functions to convert degrees to radians and vice versa
double rad2deg(double rad) { return rad * 180.0 / M_PI; }
double deg2rad(double deg) { return deg * M_PI / 180.0; }

class PreApproach : public rclcpp::Node {
    private:
    double obstacle;
    double degrees;

    bool first_run = true;
    bool reached_obstacle = false;
    bool start_rotation = false;
    int index_forward;
    double distance_forward_=5.5e15; //large number

    double current_yaw_;
    double desired_yaw_;
   
    void laser_scan_callback(const LaserScan::SharedPtr scan_msg);
    void odom_callback(const Odometry::SharedPtr odom_msg);
    void timer_callback();

    TimerBase::SharedPtr timer_;
    Subscription<LaserScan>::SharedPtr laser_sub_;
    Subscription<Odometry>::SharedPtr odom_sub_;
    Publisher<Twist>::SharedPtr cmd_pub_;

    CallbackGroup::SharedPtr timer_cb_group_;
    CallbackGroup::SharedPtr laser_cb_group_;
    CallbackGroup::SharedPtr odom_cb_group_;

    public:
    PreApproach(int &argc, char** argv);
    void publish_velocity(double linear, double angular);
};

PreApproach::PreApproach(int &argc, char** argv):Node("pre_approach_node"){
    auto pd_obstacle = rcl_interfaces::msg::ParameterDescriptor{};
    pd_obstacle.description = "Distance (in meters) to the obstacle at which the robot will stop.";
    this->declare_parameter<std::double_t>("obstacle", 0.0, pd_obstacle);
    
    auto pd_degrees = rcl_interfaces::msg::ParameterDescriptor{};
    pd_degrees.description = "Number of degrees for the rotation of the robot after stopping.";
    this->declare_parameter<std::int16_t>("degrees", 0, pd_degrees);


    this->get_parameter("obstacle", obstacle);
    RCLCPP_INFO(this->get_logger(), "Obstacle parameter: %f", obstacle);
    std::int16_t degrees_int;
    this->get_parameter("degrees", degrees_int);
    degrees = normalize_angle(deg2rad(degrees_int));
    RCLCPP_INFO(this->get_logger(), "Degrees parameter: %d", degrees_int);

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
            RCLCPP_INFO(this->get_logger(), "Shutting down...");
            RCLCPP_DEBUG(this->get_logger(), "Current yaw: %.2f, desired: %.2f",current_yaw_, desired_yaw_);
            publish_velocity(0.0, 0.0);
            rclcpp::shutdown();
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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

/*
Node: 
    - Subscribe: /scan
    - Publish: /robot/cmd_vel
Tasks:
i. Starts publishing a linear velocity to the /robot/cmd_vel topic in order to move the robot forward.
ii. When the laser detects the obstacle (the wall) x m in front of the robot, stop the robot.
iii. Then, the robot will start a rotation of x degrees.
*/
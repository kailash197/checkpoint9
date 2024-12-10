#ifndef COMPOSITION__PREAPPROACH_COMPONENT_HPP_
#define COMPOSITION__PREAPPROACH_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;

const string SCAN_TOPIC = "/scan";
const string ODOM_TOPIC = "/diffbot_base_controller/odom";
const string CMD_TOPIC = "/diffbot_base_controller/cmd_vel_unstamped";
constexpr double LINEAR_VEL = 0.50;
constexpr double OBSTACLE = 0.30;
constexpr int DEGREES = -90;


class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &options);
  void publish_velocity(double linear, double angular);

private:
  TimerBase::SharedPtr timer_;
  Subscription<LaserScan>::SharedPtr laser_sub_;
  Subscription<Odometry>::SharedPtr odom_sub_;
  Publisher<Twist>::SharedPtr cmd_pub_;

  CallbackGroup::SharedPtr timer_cb_group_;
  CallbackGroup::SharedPtr laser_cb_group_;
  CallbackGroup::SharedPtr odom_cb_group_;

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
};
//helpers
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


} // namespace my_components

#endif // COMPOSITION__PREAPPROACH_COMPONENT_HPP_
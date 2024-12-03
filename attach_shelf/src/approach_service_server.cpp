#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include <algorithm> //std::find_if()

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
// using Odometry = nav_msgs::msg::Odometry;
using GoToLoading = attach_shelf::srv::GoToLoading;
using Groups = std::vector<std::tuple<size_t, size_t, std::pair<double, double>>>;

const string SCAN_TOPIC = "/scan";
const string CMD_TOPIC = "/diffbot_base_controller/cmd_vel_unstamped";
const string SERVICE_NAME = "/approach_shelf";
constexpr double LINEAR_VEL = 0.80;
constexpr float INTENSITY_THRESHOLD = 7000;

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
    public:
        AppoarchServiceServerNode():Node("approach_shelf_server_node"){
            server_ = this->create_service<GoToLoading>(
                SERVICE_NAME,
                std::bind(&AppoarchServiceServerNode::service_callback, this, _1, _2));
            laser_cb_group_ = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
            auto laser_sub_options = SubscriptionOptions(); laser_sub_options.callback_group = laser_cb_group_;
            laser_sub_ = this->create_subscription<LaserScan>(
                SCAN_TOPIC, QoS(10).best_effort(),
                std::bind(&AppoarchServiceServerNode::laser_scan_callback, this, _1),
                laser_sub_options);
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

        if (attach_to_shelf){
            // perform final approach
            
            //ii. move robot underneathe the shelf
            //iii. lift the shelf

        } else {

        }
        // True: only if the final approach is successful
        // False: if the laser only detects 1 shelf leg or none
        response->complete = true;       
    RCLCPP_INFO(this->get_logger(), "Service Completed.");
}

void AppoarchServiceServerNode::laser_scan_callback(const LaserScan::SharedPtr scan_msg){
    auto groups = find_midpoint_intensity_groups(scan_msg, INTENSITY_THRESHOLD);
    int nof_groups = groups.size();
    if (nof_groups != 2){

    } else {
        // Both legs detected
        double cart_x = (std::get<2>(groups[0]).first + std::get<2>(groups[1]).first)/2.0;
        double cart_y = (std::get<2>(groups[0]).second + std::get<2>(groups[1]).second)/2.0;
        RCLCPP_INFO(this->get_logger(), "Cart position: (%.2f, %.2f)", cart_x, cart_y);
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
#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

// using LaserScan = sensor_msgs::msg::LaserScan;
// using Twist = geometry_msgs::msg::Twist;
// using Odometry = nav_msgs::msg::Odometry;
// using Pose2D = geometry_msgs::msg::Pose2D;
using GoToLoading = attach_shelf::srv::GoToLoading;

const string service_name = "/approach_shelf";

class AppoarchServiceServerNode : public Node {
    private:
        Service<GoToLoading>::SharedPtr server_;
        void service_callback(
            const std::shared_ptr<GoToLoading::Request> request,
            const std::shared_ptr<GoToLoading::Response> response);
    public:
        AppoarchServiceServerNode():Node("approach_shelf_server_node"){
            server_ = this->create_service<GoToLoading>(
                service_name,
                std::bind(&AppoarchServiceServerNode::service_callback, this, std::placeholders::_1, std::placeholders::_2));
            
            RCLCPP_INFO(this->get_logger(), "Service Server Ready.");
        }
};

void AppoarchServiceServerNode::service_callback(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {
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
#include "my_components/attach_client.hpp"

using namespace std::chrono_literals;
const std::string SERVICE_NAME = "/approach_shelf";

my_components::AttachClient::AttachClient(const rclcpp::NodeOptions &options):
    Node("attach_client", options) {
    service_client_ = this->create_client<GoToLoading>(SERVICE_NAME);
    timer_ = this->create_wall_timer(50ms, std::bind(&AttachClient::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "AttachClient Ready.");
}

void my_components::AttachClient::send_async_request() {
    while (!service_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = true;
    auto result_future = service_client_->async_send_request(
        request, std::bind(&AttachClient::service_response_callback, this, std::placeholders::_1));
}

void my_components::AttachClient::service_response_callback(rclcpp::Client<GoToLoading>::SharedFuture future) {
    auto status = future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Service Status: %s", response->complete ? "successful" : "failed");
        service_done_ = true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
        service_done_ = false;
    }
}
void my_components::AttachClient::timer_callback() {
    if (final_approach) {
        if (!service_requested_){
        //call service attach shelf
        RCLCPP_INFO(this->get_logger(), "Service Request Sent.");
        send_async_request();
        service_requested_ = true;
        }
        if (!service_done_) {
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Service Completed.");
            RCLCPP_INFO(this->get_logger(), "Shutting down.");
            rclcpp::shutdown();
        }
    }
} 

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
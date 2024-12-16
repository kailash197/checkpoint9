#ifndef COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "my_components/srv/go_to_loading.hpp"

using GoToLoading = my_components::srv::GoToLoading;
namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

private:
  bool final_approach = true;
  rclcpp::Client<GoToLoading>::SharedPtr service_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  bool service_requested_ = false;
  void send_async_request();
  void service_response_callback(rclcpp::Client<GoToLoading>::SharedFuture future);
  void timer_callback();
};

} // namespace my_components

#endif // COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
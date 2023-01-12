#include "rclcpp/rclcpp.hpp"
#include "roboteq_interfaces/srv/maintenance.hpp"
 
using namespace std::chrono_literals;

 int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("maintenance_client");
    auto client = node->create_client<roboteq_interfaces::srv::Maintenance>("maintenance_service",rmw_qos_profile_services_default);

    auto request = std::make_shared<roboteq_interfaces::srv::Maintenance::Request>();
    request->user_input = argv[1];

    auto res = client->async_send_request(request);

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("roboteq_maintenance_client"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("roboteq_maintenance_client"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("roboteq_maintenance_client"), "success!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("roboteq_maintenance_client"), "Failed to call service");
    }

    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interface/srv/get_direction.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using GetDirection = custom_interface::srv::GetDirection;

class TestService : public rclcpp::Node
{
public:
    TestService(const std::string &service_name) : Node("client_direction")
    {
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&TestService::laser_callback, this, _1));
        client_ = this->create_client<GetDirection>(service_name);

        // Create a timer for service calls
        timer_ = this->create_wall_timer(100ms, std::bind(&TestService::timer_callback, this));
    }

    bool is_service_done() const { return this->service_done_; }

private:
    // suscriber to laser scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    sensor_msgs::msg::LaserScan::SharedPtr data_laser_;
    // service client
    rclcpp::Client<GetDirection>::SharedPtr client_;
    bool service_done_ = false;
    // response
    std::string direction_;
    rclcpp::TimerBase::SharedPtr timer_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        data_laser_ = msg;
        // Reset the timer to ensure periodic service calls
        timer_->reset();
    }

    void timer_callback()
    {
        if (!client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(this->get_logger(), "Service Unavailable. Retrying...");
            return;
        }

        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = data_laser_;

        service_done_ = false;
        auto result_future = client_->async_send_request(
            request, std::bind(&TestService::response_callback, this,
                               std::placeholders::_1));
    }

    void response_callback(rclcpp::Client<GetDirection>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "Service Response Received !!!");
        auto status = future.wait_for(1s);
        if (status == std::future_status::ready)
        {
            auto result = future.get(); // obtain the result of the service call
            direction_ = result->direction;
            service_done_ = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed to finish properly");
            direction_ = "Fail";
        }
        RCLCPP_INFO(this->get_logger(), "DIRECTION to follow: %s", direction_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto service_client = std::make_shared<TestService>("direction_service");
    rcutils_logging_set_logger_level(service_client->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_DEBUG); // set logger level to debug

    rclcpp::spin(service_client); // Use spin instead of spin_some
    rclcpp::shutdown();
    return 0;
}

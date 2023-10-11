//
//    Remove the computation of the direction_ in the laser callback.
//    Now, in the laser callback, just store the latest laser message ina class variable named last_laser_
//    In the control loop, call the service providing the last_laser_ data
//    Based on the response obtained from the service, assign the following speeds to the robot:
//        If front then linear.x=0.1 and angular.z=0.0
//        If left then linear.x=0.1 and angular.z=0.5
//        If right then linear.x=0.1 and angular.z=-0.5
//

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interface/srv/get_direction.hpp"
#include <chrono>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using GetDirection = custom_interface::srv::GetDirection;
using std::placeholders::_1;

class Patrol : public rclcpp::Node
{
public:
    Patrol(const std::string &service_name) : Node("client_direction")
    {
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Patrol::laser_callback, this, _1));
        client_ = this->create_client<GetDirection>(service_name);
        // publish to cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // Create a timer for service calls
        timer_ = this->create_wall_timer(100ms, std::bind(&Patrol::timer_callback, this));
        // values for linear and angular speed
        linear_speed = 0.1;
        angular_speed = 0.5;
    }

    bool is_service_done() const { return this->service_done_; }

private:
    // suscriber to laser scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    sensor_msgs::msg::LaserScan::SharedPtr last_laser_;
    // service client
    rclcpp::Client<GetDirection>::SharedPtr client_;
    bool service_done_ = false;
    // response
    std::string direction_;
    rclcpp::TimerBase::SharedPtr timer_;
    // publisher to cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist pub_msg_;
    float linear_speed;
    float angular_speed;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_laser_ = msg;
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
        request->laser_data = *last_laser_;

        service_done_ = false;
        auto result_future = client_->async_send_request(
            request, std::bind(&Patrol::response_callback, this,
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
        RCLCPP_INFO(this->get_logger(), "DIRECTION to follow: %s", std::string(direction_).c_str());
        // publish to cmd_vel
        if (direction_ == "forward")
        {
            pub_msg_.linear.x = linear_speed;
            pub_msg_.angular.z = 0.0;
        }
        else if (direction_ == "left")
        {
            pub_msg_.linear.x = linear_speed;
            pub_msg_.angular.z = angular_speed;
        }
        else if (direction_ == "right")
        {
            pub_msg_.linear.x = linear_speed;
            pub_msg_.angular.z = -angular_speed;
        }
        else
        {
            pub_msg_.linear.x = 0.0;
            pub_msg_.angular.z = 0.0;
        }
        publisher_->publish(pub_msg_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto service_client = std::make_shared<Patrol>("direction_service");
    rcutils_logging_set_logger_level(service_client->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_DEBUG); // set logger level to debug

    rclcpp::spin(service_client);
    rclcpp::shutdown();
    return 0;
}

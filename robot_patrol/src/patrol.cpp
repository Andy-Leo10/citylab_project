#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <chrono>
using std::placeholders::_1;
using namespace std::chrono_literals;

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrolling_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Patrol::laser_callback, this, _1));
        timer_ = this->create_wall_timer(100ms, std::bind(&Patrol::timer_callback, this));
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        data_laser_ = msg;
        determine_MaxDistance_MaxIndex();
        RCLCPP_INFO(this->get_logger(), "Further[m]: %f, Further[°]: %f", distance_, direction_ * 180 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Closest[m]: %f, Closest[°]: %f", closest_distance_, closest_direction_ * 180 / M_PI);
    }
    void determine_MaxDistance_MaxIndex()
    {
        // Identify the largest distance ray, which is not inf
        float max_distance = 0;
        size_t max_index = 0;
        float min_distance = 100;
        size_t min_index = 0;
        //this robot has a 360 degree laser scanner, the ranges size is 720
        //the angle of the rays go from -pi to pi
        //but we want to see only the front of the robot, so ...
        for (size_t i = 359; i < 540; i++)
        {
            if (data_laser_->ranges[i] > max_distance && data_laser_->ranges[i] < 2.3)
            {
                max_distance = data_laser_->ranges[i];
                max_index = i;
            }
            if (data_laser_->ranges[i] < min_distance && data_laser_->ranges[i] > 0.1)
            {
                min_distance = data_laser_->ranges[i];
                min_index = i;
            }
        }
        distance_ = max_distance;
        closest_distance_ = min_distance;
        // we remap the index of interest to the range of -pi to pi
        direction_ = (M_PI / 360) * max_index - M_PI;
        closest_direction_ = (M_PI / 360) * min_index - M_PI;
    }
    void timer_callback()
    {
        // move following the algorithm
        move_.linear.x = this->linear_x;
        this->angular_z = direction_ * 0.5;
        move_.angular.z = this->angular_z;
        publisher_->publish(move_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist move_;
    float linear_x = 0.1;
    float angular_z = 0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    sensor_msgs::msg::LaserScan::SharedPtr data_laser_;
    float distance_ = 0;
    float direction_ = 0;
    float closest_distance_ = 100;
    float closest_direction_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}
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
        RCLCPP_INFO(this->get_logger(), "Distance[m]: %f, Direction[°]: %f", distance_, direction_ * 180 / M_PI);
    }
    void determine_MaxDistance_MaxIndex()
    {
        // Identify the largest distance ray, which is not inf
        float max_distance = 0;
        size_t max_index = 0;
        float min_distance = 100;
        size_t min_index = 0;
        for (size_t i = 0; i < data_laser_->ranges.size(); i++)
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
        // laser readings are from 0 to 719 rays, direction is from -pi/2 to pi/2
        direction_ = (M_PI / 720) * max_index - M_PI / 2;
        closest_distance_ = min_distance;
        closest_direction_ = (M_PI / 720) * min_index - M_PI / 2;
    }
    void timer_callback()
    {
        // move following the algorithm
        move_.linear.x = this->linear_x;
        this->angular_z = direction_ * 0.5;
        move_.angular.z = this->angular_z;
        //for the closest distance turn in the opposite direction
        if (closest_distance_ < 0.5)
        {
            //modify the direction
            move_.angular.z = -closest_direction_ * 0.5;
        }
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
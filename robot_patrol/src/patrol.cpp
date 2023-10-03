#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <chrono>
#include <iostream>
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
        //RCLCPP_INFO(this->get_logger(), "Further[m]: %f, Further[°]: %f", distance_, direction_ * 180 / M_PI);
        //RCLCPP_INFO(this->get_logger(), "Closest[m]: %f, Closest[°]: %f", closest_distance_, closest_direction_ * 180 / M_PI);
    }
    void determine_MaxDistance_MaxIndex()
    {
        // Identify the largest distance ray, which is not inf
        float max_distance = 0;
        size_t max_index = 0;
        float min_distance = 100;
        size_t min_index = 0;
        // this robot has a 360 degree laser scanner, the ranges size is 720
        // the angle of the rays go from -pi to pi
        // but we want to see only the front of the robot, so ...
        for (size_t i = 179; i < 540; i++)
        {
            if (data_laser_->ranges[i] > max_distance && data_laser_->ranges[i] < 2.4)
            {
                max_distance = data_laser_->ranges[i];
                max_index = i;
            }
            if (data_laser_->ranges[i] < min_distance && data_laser_->ranges[i] > 0.12)
            {
                min_distance = data_laser_->ranges[i];
                min_index = i;
            }
        }
        distance_ = max_distance;
        closest_distance_ = min_distance;
        // we remap the index of interest to the range of -pi to pi
        direction_ = (M_PI / 720) * max_index - M_PI / 2;
        closest_direction_ = (M_PI / 720) * min_index - M_PI / 2;
    }

    void check_angular_z()
    {
        // it is a saturation function for the angular velocity from [-pi/2;pi/2]
        if (this->angular_z > M_PI / 2)
            this->angular_z = M_PI / 2;
        else if (this->angular_z < -M_PI / 2)
            this->angular_z = -M_PI / 2;
    }

    void check_spaces(){
        //supposing there is free spaces
        space_left_ = true;
        space_right_ = true;
        // check if there is enough space on the left
        for (size_t i = 361; i < 461; i++)
        {
            if (data_laser_->ranges[i] < SAFE_BUBBLE)
            {
                space_left_ = false;
                break;
            }
        }
        if(space_left_)
        {
            // check if there is enough space on the right
            for (size_t i = 269; i < 359; i++)
            {
                if (data_laser_->ranges[i] < SAFE_BUBBLE)
                {
                    space_right_ = false;
                    break;
                }
            }
        }
    }

    void timer_callback()
    {
        check_spaces();
        move_.linear.x = this->linear_x;
        // if there is no obstacle in front of the robot
        // move following the algorithm
        if(space_left_ && space_right_)
        {
            this->angular_z = direction_ * 0.5;
            RCLCPP_INFO(this->get_logger(), "case 1 - far away");
        }
        // if there is no enough space on the left
        else if (!space_left_ && space_right_)
        {
            this->angular_z = this->angular_z - variation_;
            RCLCPP_INFO(this->get_logger(), "case 2 - to right");
        }
        // if there is no enough space on the right
        else if (!space_right_ && space_left_)
        {
            this->angular_z = this->angular_z + variation_;
            RCLCPP_INFO(this->get_logger(), "case 3 - to left");
        }
        check_angular_z();
        move_.angular.z = this->angular_z;
        publisher_->publish(move_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist move_;
    float linear_x = 0.1;
    float angular_z = 0.05;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    sensor_msgs::msg::LaserScan::SharedPtr data_laser_;
    float distance_ = 0;
    float direction_ = 0;
    float closest_distance_ = 100;
    float closest_direction_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    const float SAFE_BUBBLE = 0.22;
    bool space_left_ = true;
    bool space_right_ = true;
    float variation_ = 0.7;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}
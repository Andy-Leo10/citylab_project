#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interface/srv/get_direction.hpp"

#include <memory>

using GetDirection = custom_interface::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node
{
public:
    DirectionService() : Node("service_server_direction")
    {
        srv_ = create_service<GetDirection>(
            "direction_service", std::bind(&DirectionService::direction_callback, this, _1, _2));
        direction_ = "forward";
        MAX_DIST = 2.4;
    }

private:
    rclcpp::Service<GetDirection>::SharedPtr srv_;
    float MAX_DIST;
    std::string direction_;
    sensor_msgs::msg::LaserScan laser_data_;
    float total_dist_sec_right;
    float total_dist_sec_front;
    float total_dist_sec_left;

    void direction_callback(const std::shared_ptr<GetDirection::Request> request,
                            const std::shared_ptr<GetDirection::Response> response)
    {
        this->total_dist_sec_right = MAX_DIST;
        this->total_dist_sec_front = MAX_DIST;
        this->total_dist_sec_left = MAX_DIST;
        laser_data_ = request->laser_data;

        // this robot has a 360 degree laser scanner, the ranges size is 720
        // the angle of the rays go from -pi to pi
        // let's determine the minimum distance for each section
        for (size_t i = 179; i < 540; i++)
        {
            if (laser_data_.ranges[i] < MAX_DIST)
            {
                if (i < 299)
                {
                    // evaluate min distance for right section
                    if (laser_data_.ranges[i] < total_dist_sec_right)
                    {
                        total_dist_sec_right = laser_data_.ranges[i];
                    }
                }
                else if (i < 419)
                {
                    // evaluate min distance for front section
                    if (laser_data_.ranges[i] < total_dist_sec_front)
                    {
                        total_dist_sec_front = laser_data_.ranges[i];
                    }
                }
                else
                {
                    // evaluate min distance for left section
                    if (laser_data_.ranges[i] < total_dist_sec_left)
                    {
                        total_dist_sec_left = laser_data_.ranges[i];
                    }
                }
            }
        }
        // now let's determine the biggest distance
        if (total_dist_sec_right > total_dist_sec_front)
        {
            if (total_dist_sec_right > total_dist_sec_left)
                direction_ = "right";
            else
                direction_ = "left";
        }
        else
        {
            if (total_dist_sec_front > total_dist_sec_left)
                direction_ = "forward";
            else
                direction_ = "left";
        }
        response->direction = direction_;
        RCLCPP_INFO(this->get_logger(), "Service finished \n");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto server_node = std::make_shared<DirectionService>(); // create node
    rcutils_logging_set_logger_level(server_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    RCLCPP_DEBUG(server_node->get_logger(), "SERVICE = /direction_service");
    rclcpp::spin(server_node);
    rclcpp::shutdown();
    return 0;
}

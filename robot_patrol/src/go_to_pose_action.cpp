/*
Description: This is the action server for the action "go_to_pose".
The robot will move to the desired pose with a tolerance.
The action server will publish feedback about the current position of the robot.
The action server will publish the result when the robot reaches the desired pose.
*/

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_interface/action/go_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class MyActionServer : public rclcpp::Node
{
public:
    using DesirePose = custom_interface::action::GoToPose;
    using GoalHandleDesirePose = rclcpp_action::ServerGoalHandle<DesirePose>;

    explicit MyActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("go_to_server", options), TOLERANCE_(0.1), MAX_ANGULAR_SPEED_(M_PI / 2)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<DesirePose>(
            this,
            "go_to_pose",
            std::bind(&MyActionServer::handle_goal, this, _1, _2),
            std::bind(&MyActionServer::handle_cancel, this, _1),
            std::bind(&MyActionServer::handle_accepted, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MyActionServer::odomCallback, this, _1));
        desired_pos_.x = 0.0;
        desired_pos_.y = 0.0;
        desired_pos_.theta = 0.0;
        direction_ = 0.0;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        this->linear_speed = 0.2;
        this->angular_speed = 0.5;
    }

private:
    // definition of action server
    rclcpp_action::Server<DesirePose>::SharedPtr action_server_;
    // definition of odom subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // for storing the desired position, current position and the difference between them
    geometry_msgs::msg::Pose2D desired_pos_;
    geometry_msgs::msg::Pose2D current_pos_;
    float diff_x_, diff_y_, diff_theta_;
    const float TOLERANCE_;
    float direction_;
    // for publishing to cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist pub_msg_;
    float linear_speed;
    float angular_speed;
    float MAX_ANGULAR_SPEED_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // store the current position
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;
        // convert the quaternion into Euler angles
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw, 1);
        // store the current theta
        current_pos_.theta = yaw;

        // compute the difference between the current position and the desired position
        diff_x_ = desired_pos_.x - current_pos_.x;
        diff_y_ = desired_pos_.y - current_pos_.y;
        // the desired angle is in degrees, so we need to convert it to radians
        diff_theta_ = desired_pos_.theta * M_PI / 180 - current_pos_.theta;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const DesirePose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose goal request with x: %f, y: %f, theta: %f", goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDesirePose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDesirePose> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleDesirePose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1); // Hz for each control loop
        auto feedback = std::make_shared<DesirePose::Feedback>();
        auto &goal = goal_handle->get_goal();
        auto result = std::make_shared<DesirePose::Result>();
        // control loop to achive the desired pose with a tolerance
        while (rclcpp::ok())
        {
            // check if there is a cancel request
            if (goal_handle->is_canceling())
            {
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                break;
            }

            // check if the goal is done
            if (fabs(diff_x_) < TOLERANCE_ && fabs(diff_y_) < TOLERANCE_ && fabs(diff_theta_) < TOLERANCE_)
            {
                result->status = true;
                goal_handle->succeed(result);
                stop();
                RCLCPP_INFO(this->get_logger(), "Goal Succeeded\n");
                break;
            }

            // compute the direction of the vector between the current position and the desired position
            direction_ = atan2(diff_y_, diff_x_);
            // compute the angular speed
            this->angular_speed = direction_ * 0.5;
            // debug info
            RCLCPP_DEBUG(this->get_logger(), "Current direction: %f", direction_*180/M_PI);
            RCLCPP_DEBUG(this->get_logger(), "Current theta: %f \n", current_pos_.theta*180/M_PI);

            // publish the velocity command
            check_angular_speed();
            pub_msg_.linear.x = this->linear_speed;
            pub_msg_.angular.z = this->angular_speed;
            publisher_->publish(pub_msg_);

            // publish feedback
            feedback->current_pos.x = current_pos_.x;
            feedback->current_pos.y = current_pos_.y;
            // convert the theta from radians to degrees
            feedback->current_pos.theta = current_pos_.theta * 180 / M_PI;
            goal_handle->publish_feedback(feedback);

            // sleep for the time that was defined for the control loop
            loop_rate.sleep();
        }
    }

    void stop()
    {
        pub_msg_.linear.x = 0.0;
        pub_msg_.angular.z = 0.0;
        publisher_->publish(pub_msg_);
    }
    void check_angular_speed()
    {
        if (this->angular_speed > MAX_ANGULAR_SPEED_)
            this->angular_speed = MAX_ANGULAR_SPEED_;
        else if (this->angular_speed < -MAX_ANGULAR_SPEED_)
            this->angular_speed = -MAX_ANGULAR_SPEED_;
    }
}; // class MyActionServer

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<MyActionServer>();
    rcutils_ret_t ret = rcutils_logging_set_logger_level(action_server->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
        std::cout<< "----------------------------handle the error [main CODE]";// handle the error
    }
    RCLCPP_INFO(action_server->get_logger(), "ACTION = /go_to_pose READY!");
    RCLCPP_INFO(action_server->get_logger(), "FOR TEST USE = ros2 action send_goal -f /go_to_pose custom_interface/action/GoToPose \"goal_pos:  x: 0.7 y: 0.3 theta: 0.0\"");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

/*
ros2 action send_goal -f /go_to_pose custom_interface/action/GoToPose "goal_pos:
  x: 0.7
  y: 0.3
  theta: 0.0
"
*/
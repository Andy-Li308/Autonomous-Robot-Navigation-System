#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    // Callback for path
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // Callback for odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Main loop to continuously follow the path
    void followPath();

    // Timer callback
    void timerCallback();

    //quaternion to yaw conversion
    double quaternionToYaw(double x, double y, double z, double w);

  private:
    robot::ControlCore control_;

    // Subscriber and Publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // robot state
    double robot_x_;
    double robot_y_;
    double robot_theta_;

    //controller parameters
    double lookahead_distance_ = 1.0; //meters
    double linear_velocity_ = 1.5;
    double max_steering_angle_ = 1.5;
};

#endif

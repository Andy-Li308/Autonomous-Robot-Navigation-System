#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void updatePath(nav_msgs::msg::Path path);

    int findLookaheadPoint(double robot_x, double robot_y, double robot_theta);

    geometry_msgs::msg::Twist calculateControlCommand(double robot_x, double robot_y, double robot_theta);

    bool isPathEmpty();
  
  private:
    nav_msgs::msg::Path current_path_;
    rclcpp::Logger logger_;

    double lookahead_distance_ = 1.0; //meters
    double linear_velocity_ = 1.5; // m/s
    double max_steering_angle_ = 1.5; // radians
    double goal_tolerance_ = 0.2; // meters
};

} 

#endif 

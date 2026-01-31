#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr  goal_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void timerCallback();

    void publishPath();
    void resetGoal();

  private:
    robot::PlannerCore planner_;

    //subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

 // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Costmap 
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;

    // Current goal
    geometry_msgs::msg::PointStamped current_goal_;
    bool active_goal_;
    rclcpp::Time plan_start_time_;

    // Robot odometry (x,y). For simplicity, ignoring orientation usage here.
    bool have_odom_;
    double odom_x_;
    double odom_y_;

    //node parameters

    double goal_tolerance_ = 0.3;
    double plan_timeout_ = 30.0; //seconds
};

#endif 

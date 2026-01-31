#ifndef MAP_MEMORY_NODE_HPP
#define MAP_MEMORY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();

private:
    void handleLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishGlobalMap();
    double quaternionToYaw(double x, double y, double z, double w);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::shared_ptr<robot::MapMemoryCore> map_memory_;

    double robot_position_x_, robot_position_y_, robot_orientation_theta_;
    double previous_position_x_, previous_position_y_;
    double movement_threshold_;
};

#endif
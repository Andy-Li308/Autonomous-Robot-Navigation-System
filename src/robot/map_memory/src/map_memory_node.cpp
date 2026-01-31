#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory_node"), robot_position_x_(0), robot_position_y_(0),
    robot_orientation_theta_(0), previous_position_x_(0), previous_position_y_(0), movement_threshold_(5) {

    map_memory_ = std::make_shared<robot::MapMemoryCore>(this->get_logger());

    local_map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::handleLocalMap, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::handleOdometry, this, std::placeholders::_1));

    global_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    publish_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::publishGlobalMap, this));

    geometry_msgs::msg::Pose map_origin;

    map_origin.position.x = -15.0;
    map_origin.position.y = -15.0;
    map_origin.orientation.w = 1.0;

    map_memory_->initializeMap(0.1, 300, 300, map_origin);

    RCLCPP_INFO(this->get_logger(), "Map memory node initialized.");
}

void MapMemoryNode::handleLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty local map.");
        return;
    }

    double distance_moved = std::hypot(robot_position_x_ - previous_position_x_, 
                                       robot_position_y_ - previous_position_y_);

    previous_position_x_ = robot_position_x_;
    previous_position_y_ = robot_position_y_;

    map_memory_->integrateLocalMap(msg, robot_position_x_, robot_position_y_, robot_orientation_theta_);
}

void MapMemoryNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_position_x_ = msg->pose.pose.position.x;
    robot_position_y_ = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    robot_orientation_theta_ = quaternionToYaw(qx, qy, qz, qw);
}

void MapMemoryNode::publishGlobalMap() {
    auto global_map = map_memory_->getGlobalMap();
    global_map->header.stamp = this->now();
    global_map->header.frame_id = "sim_world";
    global_map_publisher_->publish(*global_map);
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
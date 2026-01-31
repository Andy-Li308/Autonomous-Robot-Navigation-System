#ifndef MAP_MEMORY_CORE_HPP
#define MAP_MEMORY_CORE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace robot {

class MapMemoryCore {
public:
    MapMemoryCore(const rclcpp::Logger& logger);

    void initializeMap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin);

    // Method to integrate the local map based on the robot's movement
    void integrateLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& local_map,
                            double robot_x, double robot_y, double robot_theta);

    nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalMap() const;

private:
    bool mapCoordinatesToGlobal(double world_x, double world_y, int& global_x, int& global_y) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    std::vector<bool> updated_cells_;

    // Track the robot's last position
    double last_robot_x_;
    double last_robot_y_;

    // The threshold for updating the map (5 meters)
    double move_threshold_;

    // Helper method to compute the distance between two points
    double computeDistance(double x1, double y1, double x2, double y2) const;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP

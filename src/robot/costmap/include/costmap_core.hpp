#ifndef COSTMAP_CORE_HPP
#define COSTMAP_CORE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
class CostmapCore {
public:
    CostmapCore();
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_, const sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg);  

private:
    std::vector<std::vector<int>> costmap_grid_;
    const int costmap_width_ = 300;  // width (number of cells)
    const int costmap_height_ = 300; // height (number of cells)

    const int x_center = costmap_width_ / 2;
    const int y_center = costmap_height_ / 2; //puts the robot in the center of the costmap grid (bc the vector cant have negative indeces)

    const double resolution_ = 0.1;  // resolution in meters per cell
    const int inflation_radius_ = 1; //inflation radius in meters

    
};

#endif 
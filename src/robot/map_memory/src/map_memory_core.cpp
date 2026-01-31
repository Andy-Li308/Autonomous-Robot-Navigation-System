#include "map_memory_core.hpp"
#include <cmath> 

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
    : logger_(logger), last_robot_x_(0), last_robot_y_(0), move_threshold_(2.0) {  // 2 meters threshold
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
}

void MapMemoryCore::initializeMap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin) {
    global_map_->info.resolution = resolution;
    global_map_->info.width = width;
    global_map_->info.height = height;
    global_map_->info.origin = origin;
    global_map_->data.resize(width * height, 0);

    updated_cells_.resize(width * height, false); 


    last_robot_x_ = origin.position.x;
    last_robot_y_ = origin.position.y;

    RCLCPP_INFO(logger_, "Initialized global map");
}

void MapMemoryCore::integrateLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& local_map,
                                      double robot_x, double robot_y, double robot_theta) {
    // distance moved since last update

    double distance_moved = computeDistance(last_robot_x_, last_robot_y_, robot_x, robot_y);

    if (distance_moved >= move_threshold_) {
        for (unsigned int y = 0; y < local_map->info.height; ++y) {
            for (unsigned int x = 0; x < local_map->info.width; ++x) {
                int index = y * local_map->info.width + x;
                if (local_map->data[index] < 0) continue; 

                double local_x = local_map->info.origin.position.x + x * local_map->info.resolution;
                double local_y = local_map->info.origin.position.y + y * local_map->info.resolution;

                double world_x = robot_x + local_x * std::cos(robot_theta) - local_y * std::sin(robot_theta);
                double world_y = robot_y + local_x * std::sin(robot_theta) + local_y * std::cos(robot_theta);

                int global_x, global_y;
                if (mapCoordinatesToGlobal(world_x, world_y, global_x, global_y)) {
                    int global_index = global_y * global_map_->info.width + global_x; 

  
                    if (!updated_cells_[global_index]) {
                        global_map_->data[global_index] = local_map->data[index];
                        updated_cells_[global_index] = true;  // cell as updated
                    }
                    else {
                        global_map_->data[global_index] = std::max(global_map_->data[global_index], local_map->data[index]);
                    }
                }
            }
        }

        // update the robot position
        last_robot_x_ = robot_x;
        last_robot_y_ = robot_y;
    }
}

double MapMemoryCore::computeDistance(double x1, double y1, double x2, double y2) const {
    // euclidean distance
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

bool MapMemoryCore::mapCoordinatesToGlobal(double world_x, double world_y, int& global_x, int& global_y) const {
    double origin_x = global_map_->info.origin.position.x;
    double origin_y = global_map_->info.origin.position.y;
    double resolution = global_map_->info.resolution;

    if (world_x < origin_x || world_y < origin_y) return false;

    global_x = static_cast<int>((world_x - origin_x) / resolution);
    global_y = static_cast<int>((world_y - origin_y) / resolution);

    if (global_x < 0 || global_x >= static_cast<int>(global_map_->info.width) ||
        global_y < 0 || global_y >= static_cast<int>(global_map_->info.height)) {
        return false;
    }

    return true;
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getGlobalMap() const {
    return global_map_;
}

}
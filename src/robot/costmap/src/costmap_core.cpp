#include "costmap_core.hpp"

CostmapCore::CostmapCore(){}

void CostmapCore::initializeCostmap() {
    costmap_grid_.clear();
    costmap_grid_.resize(costmap_height_, std::vector<int>(costmap_width_, 0));
}

//converts a laser scan point (range, angle) to cartesian grid coordinates (x_grid, y_grid) in the reference frame of the robot
//NOTE: the robot ref frame is both shifted in terms of the global frame and ALSO MIGHT BE ROTATED 
void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid){
    double x_value = range * std::cos(angle);
    double y_value = range * std::sin(angle);

    //converts to grid coordinates adding in the offset of the robot in the costmap (bc the vector cant have negative indexes) 
    //basically think of the robot as sitting at the coords (150,150) or whatever the center of the costmap is
    //then, the actual coord of obstacle cell is the calculated x and y coords (distance coords/resolution) + coords of the robot (150,150) 
    x_grid = static_cast<int>(x_value / resolution_ + x_center); 
    y_grid = static_cast<int>(y_value / resolution_ + y_center);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {

    if (x_grid >= 0 && x_grid < costmap_width_ && y_grid >= 0 && y_grid < costmap_height_) {
        costmap_grid_[y_grid][x_grid] = 100; // Mark as occupied, note max cost = 100 here
        //will this result in errors if the coords are outside of the size of the vector?
    }
}

void CostmapCore::inflateObstacles() {
    const int max_cost = 100; 
    int cell_inflation_radius = static_cast<int>(inflation_radius_ / resolution_);//inflation radius in terms of number of cells

    for (size_t y = 0; y < costmap_grid_.size(); y++){//loop that iterates through each row in the costmap grid
        for (size_t x = 0; x < costmap_grid_[y].size(); x++){//iterates through each cell in the selected row
            if (costmap_grid_[y][x] == max_cost){//apply inflation only to cells that are marked as obstacles

                //calculate all cells within a certain radius(the inflation radius) from the selected cell 
                for (int delta_x = x - cell_inflation_radius; delta_x <= x + cell_inflation_radius; delta_x++){
                    for (int delta_y = y - cell_inflation_radius; delta_y <= y + cell_inflation_radius; delta_y++){//scan through all cells in a square with length of "inflation diameter" centered around the obstacle cell

                        //check to make sure cell is within bounds of the costmap (both x and y coords are within bounds of costmap width and height)
                        if(delta_x >= 0 && delta_x < costmap_width_ && delta_y >= 0 && delta_y < costmap_height_){
                            //calculate euclidian distance form obstacle cell to inflation cell
                            double cell_length_distance = std::sqrt(std::pow(x - delta_x, 2) + std::pow(y - delta_y, 2));
                            double actual_distance = cell_length_distance * resolution_; //convert to meters

                            if(actual_distance <= inflation_radius_){//only calculate/assign costs to cells within inflation radius
                                int calculated_cost = static_cast<int>(max_cost * (1 - (actual_distance / inflation_radius_)));
                                costmap_grid_[delta_y][delta_x] = std::max(costmap_grid_[delta_y][delta_x], calculated_cost);
                            }
                        }
                    }
                }
            }
        }
    }
}

void CostmapCore::publishCostmap(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_, const sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg) {  
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;

    //fill in header info
    occupancy_grid_msg.header = laser_scan_msg->header; //set the header to whatever the laser scan message header is
    occupancy_grid_msg.header.stamp = rclcpp::Clock().now();

    //fill in Map metadata info
    occupancy_grid_msg.info.resolution = resolution_;
    occupancy_grid_msg.info.width = costmap_width_;
    occupancy_grid_msg.info.height = costmap_height_;

    //this is set bc the "origin" of the vector costmap (ie the cell at vector index [0][0]) is actually representing the point (-150,-150) or whatever the negative of the offset applied is
    occupancy_grid_msg.info.origin.position.x = - x_center * resolution_; 
    occupancy_grid_msg.info.origin.position.y = - y_center * resolution_; 

    occupancy_grid_msg.info.origin.orientation.x = 0.0;
    occupancy_grid_msg.info.origin.orientation.y = 0.0;
    occupancy_grid_msg.info.origin.orientation.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.w = 1.0;

    //resize costmap to 1d array
    occupancy_grid_msg.data.resize(costmap_width_ * costmap_height_);

    //fill in the flattened array with costmap data from the 2d vector
    for (int y = 0; y < costmap_height_; y++){
        for (int x = 0; x < costmap_width_; x++){
            occupancy_grid_msg.data[y * costmap_width_ + x] = costmap_grid_[y][x];
        }
    }

    //publish the costmap
    costmap_pub_->publish(occupancy_grid_msg);
}


#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : path_(std::make_shared<nav_msgs::msg::Path>()), map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

bool PlannerCore::poseToMap(double wx, double wy, CellIndex &out_idx) 
{
    // Convert world coordinates to map indices
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double resolution = map_->info.resolution;

    int mx = static_cast<int>((wx - origin_x) / resolution);
    int my = static_cast<int>((wy - origin_y) / resolution);

    // Check if indices are within map bounds
    if (mx < 0 || mx >= static_cast<int>(map_->info.width) || my < 0 || my >= static_cast<int>(map_->info.height)) {
        RCLCPP_WARN(logger_, "World coordinates (%.2f, %.2f) are out of map bounds.", wx, wy);
        return false;
    }

    out_idx.x = mx;
    out_idx.y = my;
    return true;
}

bool PlannerCore::mapToPose(const CellIndex &idx, double &wx, double &wy) 
{
    // Convert map indices to world coordinates
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double resolution = map_->info.resolution;

    wx = origin_x + (idx.x + 0.5) * resolution; // center of the cell
    wy = origin_y + (idx.y + 0.5) * resolution; // center of the cell
    return true;
}

bool PlannerCore::planPath(double start_world_x, double start_world_y, double goal_x,  double goal_y, nav_msgs::msg::OccupancyGrid::SharedPtr map) 
{
    //store the map in the member variable
    map_ = map;

    CellIndex start_idx, goal_idx;

    //convert start and end from world coordinates to map indices
    if (!poseToMap(start_world_x, start_world_y, start_idx)) {
        RCLCPP_ERROR(logger_, "Failed to convert start pose to map indices.");
        return false;
    }
    if (!poseToMap(goal_x, goal_y, goal_idx)) {
        RCLCPP_ERROR(logger_, "Failed to convert goal pose to map indices.");
        return false;
    }

    RCLCPP_INFO(logger_, "Start indices: (%d, %d), Goal indices: (%d, %d)", start_idx.x, start_idx.y, goal_idx.x, goal_idx.y);

    //run A* to find best path
    std::vector<CellIndex> path_indices; //vector to hold the resulting calculated path as map indices

    if (!doAStar(start_idx, goal_idx, path_indices)) {
        RCLCPP_ERROR(logger_, "A* failed to find a path.");
        return false;
    }

    //convert path cells to path message
    path_->poses.clear();
    path_->header.frame_id = map_->header.frame_id;
    path_->header.stamp = rclcpp::Clock().now();

    for (const CellIndex &idx : path_indices) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "sim_world";
        double wx, wy;
        mapToPose(idx, wx, wy);
        pose_stamped.pose.position.x = wx;
        pose_stamped.pose.position.y = wy;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0; //neutral orientation
        path_->poses.push_back(pose_stamped);
    }

    RCLCPP_INFO(logger_, "Path planned with %zu waypoints.", path_->poses.size());
    return true;
}

bool PlannerCore::doAStar(const CellIndex &start_idx, const CellIndex &goal_idx, std::vector<CellIndex> &out_path) 
{
    // A* algorithm implementation

    // dictionaries to hold gScores and fScores for cells
    std::unordered_map<CellIndex, double, CellIndexHash> g_cost;
    std::unordered_map<CellIndex, double, CellIndexHash> h_cost;
    std::unordered_map<CellIndex, double, CellIndexHash> f_cost;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from; //to keep track of each cell's parent for path reconstruction

    //minheap priority queue for the open set of unexplored nodes
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::vector<CellIndex> open_set_indices; //vector to hold the indices of cells in the open set for easy lookup

    //vector to hold the explored nodes
    std::vector<CellIndex> closed_set;

    //initialize start cell index
    g_cost[start_idx] = 0;
    h_cost[start_idx] = euclideanHeuristic(start_idx, goal_idx);
    f_cost[start_idx] = h_cost[start_idx] + g_cost[start_idx];

    //add the start cell to the open set as an AStarNode
    open_set.push(AStarNode(start_idx, f_cost[start_idx]));
    open_set_indices.push_back(start_idx);

    //main A* loop
    while (!open_set.empty()) {//as long as the open set has items, that means that there is still nodes to be explored meaning the algorithm should keep running

        //go to the node in open set with the lowest f cost, remove it from the open set, and add it to the closed set
        AStarNode current_node = open_set.top();
        open_set.pop();
        open_set_indices.erase(std::find(open_set_indices.begin(), open_set_indices.end(), current_node.index));
        closed_set.push_back(current_node.index);

        if (current_node.index == goal_idx) {//check if the current index is the target index
            //goal reached, reconstruct path
            RCLCPP_INFO(logger_, "Goal reached, reconstructing path.");
            
            //reconstruct path from came_from map
            out_path.clear();
            CellIndex current = current_node.index;
            out_path.push_back(current);

            auto it = came_from.find(current);

            while (it != came_from.end()) {
                current = it->second;
                out_path.push_back(current);
                it = came_from.find(current);
            }
            std::reverse(out_path.begin(), out_path.end()); //reverse the path to get it from start to goal

            return true;
        }

        //get current g cost
        double current_g = g_cost[current_node.index];

        for (const CellIndex &neighbor : getNeighbors8(current_node.index)) {
            //check if neighbor is in closed set OR if the cell has an obstacle (nontraversable)
            //if the cell value on the costmap is greater than 10, treat the cell as nontraversable 
            if (getCostmapValue(neighbor) > 10 || std::find(closed_set.begin(), closed_set.end(), neighbor) != closed_set.end()) {
                continue; //skip this neighbor
            }
            //calculate tentative g cost for the neighbor
            double tentative_g = current_g + euclideanHeuristic(current_node.index, neighbor);//get the cost of moving from node to its neighbor which in this case is equal to the distance between them 

            //check against old g cost
            if (g_cost.find(neighbor) == g_cost.end() || tentative_g < g_cost[neighbor]) {
                //if this path to neighbor is better than any previous one or there is no previous path, record it
                g_cost[neighbor] = tentative_g;
                h_cost[neighbor] = euclideanHeuristic(neighbor, goal_idx);
                f_cost[neighbor] = g_cost[neighbor] + h_cost[neighbor];

                //add a part here to keep track of each nodes parent for path reconstruction later  
                //if neighbor not in open set, add it
                if (std::find(open_set_indices.begin(), open_set_indices.end(), neighbor) == open_set_indices.end()) {
                    open_set.push(AStarNode(neighbor, f_cost[neighbor]));
                    open_set_indices.push_back(neighbor);

                    came_from[neighbor] = current_node.index; //record the parent of the neighbor
                }
            } 
        }

        
        //calculate f scores for the current cells neighbors
    }
    RCLCPP_WARN(logger_, "Open set exhausted, no path found.");
    return false; //no path found
 
} 
std::vector<CellIndex> PlannerCore::getNeighbors8(const CellIndex &c) 
{
    // returns the 8-direction neighbors indices of a cell index
    std::vector<CellIndex> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {

            if (dx == 0 && dy == 0) continue; //skip the current cell

            if (c.x + dx < 0 || c.x + dx >= static_cast<int>(map_->info.width) ||
                c.y + dy < 0 || c.y + dy >= static_cast<int>(map_->info.height)) {
                continue; //skip out-of-bounds neighbors
            }

            neighbors.push_back(CellIndex{c.x + dx, c.y + dy});
        }
    }
    return neighbors;
}

double PlannerCore::getCostmapValue(const CellIndex &idx)
{
    //check for bounds
    const int width = static_cast<int>(map_->info.width);
    const int height = static_cast<int>(map_->info.height);
    if (idx.x < 0 || idx.x >= width || idx.y < 0 || idx.y >= height) {
        return 100.0; // treat out-of-bounds as obstacle/high cost
    }

    // Get the costmap value at the given cell index
    int index = idx.y * map_->info.width + idx.x;
    return map_->data[index];
}

double PlannerCore::euclideanHeuristic(const CellIndex &a, const CellIndex &b) 
{
    // Euclidean distance heuristic
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

nav_msgs::msg::Path::SharedPtr PlannerCore::getPath() const {
  return path_;
}



}  // namespace robot

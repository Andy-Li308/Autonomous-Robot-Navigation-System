#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
  map_ = map_msg;

  if (active_goal_) {
  double elapsed = (now() - plan_start_time_).seconds();
  if (elapsed <= plan_timeout_) {
    RCLCPP_INFO(this->get_logger(), 
                "Map updated => Replanning for current goal (time elapsed: %.2f).",
                elapsed);
    publishPath();
  }
  }

}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr  goal_msg) {
  if (active_goal_) {
    RCLCPP_WARN(this->get_logger(), "Ignoring new goal; a goal is already active.");
    return;
  }
  
  if (!map_) {
    RCLCPP_WARN(this->get_logger(), "No costmap available yet. Cannot set goal.");
    return;
  }

  current_goal_ = *goal_msg;
  active_goal_ = true;
  plan_start_time_ = now();

  RCLCPP_INFO(this->get_logger(), "Received new goal: (%.2f, %.2f)",
              goal_msg->point.x, goal_msg->point.y);

  publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  // Store the latest odometry in (odom_x_, odom_y_)
  //ignore z and orientation for simplicity
  odom_x_ = odom_msg->pose.pose.position.x;
  odom_y_ = odom_msg->pose.pose.position.y;
  have_odom_ = true;
}


void PlannerNode::timerCallback() {
  if (active_goal_) {
  
    // Check if we've timed out
    double elapsed = (now() - plan_start_time_).seconds();
    if (elapsed > plan_timeout_) {
      RCLCPP_WARN(this->get_logger(), "Plan timed out after %.2f seconds. Resetting goal.", elapsed);
      resetGoal();
      return;
    }

    // Check if we reached the goal
    double distance = sqrt(pow(odom_x_ - current_goal_.point.x, 2) +  pow(odom_y_ - current_goal_.point.y, 2));
    if (distance < goal_tolerance_) {
      RCLCPP_WARN(this->get_logger(), "Plan succeeded! Elapsed Time: %.2f", elapsed);
      resetGoal();
      return;
    }
  }
}

void PlannerNode::publishPath() {
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry received yet. Cannot plan.");
    resetGoal();
    return;
  }

  // Use the robot's odometry as the start pose
  double start_world_x = odom_x_;
  double start_world_y = odom_y_;

  

    if(!planner_.planPath(start_world_x, start_world_y, current_goal_.point.x, current_goal_.point.y, map_)) {
      RCLCPP_ERROR(this->get_logger(), "Plan Failed.");
      resetGoal();
      return;
    }
  

  nav_msgs::msg::Path path_msg = *planner_.getPath();
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = map_->header.frame_id;
  
  path_pub_->publish(path_msg);
}

void PlannerNode::resetGoal() {
  active_goal_ = false;
  RCLCPP_INFO(this->get_logger(), "Resetting active goal.");

  // Publish an empty path, which should tell the robot to stop or have no path
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = this->now();

  // Use the costmap frame if available; otherwise a default like "sim_world"
    if (map_) {
      empty_path.header.frame_id = map_->header.frame_id;
    } 
    else {
      empty_path.header.frame_id = "sim_world";
    }
  
  // Publish empty path
  path_pub_->publish(empty_path);

  RCLCPP_INFO(this->get_logger(), "Robot stopped.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

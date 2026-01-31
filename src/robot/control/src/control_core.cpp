#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : current_path_(nav_msgs::msg::Path()), logger_(logger) {}



void ControlCore::updatePath(nav_msgs::msg::Path path) {
  current_path_ = path;
}

bool ControlCore::isPathEmpty() {
  return current_path_.poses.empty();
}

int ControlCore::findLookaheadPoint(double robot_x, double robot_y, double robot_theta) {
  
  double min_distance = std::numeric_limits<double>::max(); //distance for a point to be considered the closest, initialized to max
  int lookahead_index = 0; //index of the lookahead point in the path
  bool found_forward_point = false;

  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    double path_x = current_path_.poses[i].pose.position.x;
    double path_y = current_path_.poses[i].pose.position.y;

    double dx = path_x - robot_x;
    double dy = path_y - robot_y;

    double distance = std::hypot(dx, dy);

    if (distance < lookahead_distance_) continue;

    double angle_to_point = std::atan2(dy, dx);
    double angle_diff = angle_to_point - robot_theta;

    //normalize angle to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    //check if the point is in front of the robot (angle diff in [-pi/2, pi/2])
    if (std::abs(angle_diff) < M_PI / 2.0) {
      if (distance < min_distance) {
        min_distance = distance;
        lookahead_index = i;
        found_forward_point = true;
      }
      
    }
  }



  if (!found_forward_point) {
    //find the closest point if no forward point found 

    for (size_t i = 0; i < current_path_.poses.size(); ++i) {
      double path_x = current_path_.poses[i].pose.position.x;
      double path_y = current_path_.poses[i].pose.position.y;

      double dx = path_x - robot_x;
      double dy = path_y - robot_y;

      double distance = std::hypot(dx, dy);

      // if (distance < lookahead_distance_) {
      //   continue; 
      // }

      if (distance < min_distance) {
        min_distance = distance;
        lookahead_index = i;
      }
    }
  }
  return lookahead_index;
}

geometry_msgs::msg::Twist ControlCore::calculateControlCommand(double robot_x, double robot_y, double robot_theta) {
  geometry_msgs::msg::Twist cmd_vel;

  // Stop if we're within goal tolerance of the final goal pose
  auto goal_pose = current_path_.poses.back().pose.position;
  double goal_dx = goal_pose.x - robot_x;
  double goal_dy = goal_pose.y - robot_y;
  double goal_dist = std::hypot(goal_dx, goal_dy);
  double angle_to_goal = std::atan2(goal_dy, goal_dx);

  if (goal_dist < goal_tolerance_) {
    RCLCPP_INFO(logger_, "Goal reached (dist=%.3f < tol=%.3f). Stopping.", goal_dist, goal_tolerance_);
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel; // zero velocities
  }
  // If the goal is behind the robot, rotate in place
  if (std::abs(angle_to_goal - robot_theta) > M_PI / 2.0) {
    RCLCPP_INFO(logger_, "Goal is behind the robot. Rotating in place.");
    cmd_vel.linear.x = 0.0;
    double angle_diff = angle_to_goal - robot_theta;
    //normalize angle to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    double k_angular = 1.0; //gain for angular velocity
    cmd_vel.angular.z = k_angular * angle_diff;
    return cmd_vel;
  }


  int lookahead_index = findLookaheadPoint(robot_x, robot_y, robot_theta);
  if (lookahead_index >= static_cast<int>(current_path_.poses.size())) {
    RCLCPP_WARN(logger_, "Lookahead index out of bounds.");
    return cmd_vel; //return zero velocities
  }

  double lookahead_x = current_path_.poses[lookahead_index].pose.position.x;
  double lookahead_y = current_path_.poses[lookahead_index].pose.position.y;

  double dx = lookahead_x - robot_x;
  double dy = lookahead_y - robot_y;

  double angle_to_lookahead = std::atan2(dy, dx);
  double angle_diff = angle_to_lookahead - robot_theta;

  //normalize angle to [-pi, pi]
  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

  while(std::abs(angle_diff) > M_PI / 2.0) {//if the point is behind the robot, rotate until facing it
    double k_angular = 1.0; //gain for angular velocity

    double angular_velocity = k_angular * angle_diff;
    cmd_vel.angular.z = angular_velocity;
    cmd_vel.linear.x = 0.0; //no forward movement while rotating
    return cmd_vel;
  }

  //if angle diff is too large, dont move forward and just rotate 
  if (std::abs(angle_diff) > max_steering_angle_) {
    cmd_vel.linear.x = 0.0;
  }
  else {
    cmd_vel.linear.x = linear_velocity_;
  }

  //limit max steering angle
  angle_diff = std::max(-max_steering_angle_, std::min(max_steering_angle_, angle_diff)); 

  // Proportional controller for angular velocity
  double k_angular = 1.0; //gain for angular velocity
  double angular_velocity = k_angular * angle_diff;

  cmd_vel.angular.z = angular_velocity;

  return cmd_vel;


}  

}  // namespace robot

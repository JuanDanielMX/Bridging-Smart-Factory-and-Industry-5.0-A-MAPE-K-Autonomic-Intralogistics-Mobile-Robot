#include <autonomic_turtlebot3/rrt_global_planner.h>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <utility>

namespace autonomic_turtlebot3 {

RRTGlobalPlanner::RRTGlobalPlanner()
    : initialized_(false),
      allow_unknown_(false),
      lethal_cost_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
      goal_bias_(0.15),
      goal_tolerance_(0.20),
      step_size_(0.20),
      max_iterations_(6000),
      costmap_(nullptr),
      rng_(std::random_device{}()) {}

RRTGlobalPlanner::RRTGlobalPlanner(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : RRTGlobalPlanner() {
  initialize(std::move(name), costmap_ros);
}

void RRTGlobalPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("RRTGlobalPlanner has already been initialized");
    return;
  }
  if (costmap_ros == nullptr) {
    ROS_ERROR("RRTGlobalPlanner received a null costmap");
    return;
  }

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("allow_unknown", allow_unknown_, false);
  private_nh.param("goal_bias", goal_bias_, 0.15);
  private_nh.param("goal_tolerance", goal_tolerance_, 0.20);
  private_nh.param("step_size", step_size_, 0.20);
  private_nh.param("max_iterations", max_iterations_, 6000);

  int lethal_cost = static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  private_nh.param("lethal_cost", lethal_cost, lethal_cost);
  lethal_cost_ = static_cast<unsigned char>(std::max(1, std::min(255, lethal_cost)));

  goal_bias_ = std::max(0.0, std::min(1.0, goal_bias_));
  goal_tolerance_ = std::max(0.01, goal_tolerance_);
  step_size_ = std::max(0.01, step_size_);
  max_iterations_ = std::max(100, max_iterations_);

  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1, true);
  initialized_ = true;

  ROS_INFO("RRTGlobalPlanner initialized in frame %s", global_frame_.c_str());
}

bool RRTGlobalPlanner::cellIsFree(unsigned int x, unsigned int y) const {
  if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) {
    return false;
  }
  const unsigned char cost = costmap_->getCost(x, y);
  if (cost == costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < lethal_cost_;
}

bool RRTGlobalPlanner::segmentIsFree(unsigned int x0, unsigned int y0,
                                     unsigned int x1, unsigned int y1) const {
  int x = static_cast<int>(x0);
  int y = static_cast<int>(y0);
  const int target_x = static_cast<int>(x1);
  const int target_y = static_cast<int>(y1);
  const int dx = std::abs(target_x - x);
  const int sx = x < target_x ? 1 : -1;
  const int dy = -std::abs(target_y - y);
  const int sy = y < target_y ? 1 : -1;
  int error = dx + dy;

  while (true) {
    if (x < 0 || y < 0 ||
        !cellIsFree(static_cast<unsigned int>(x), static_cast<unsigned int>(y))) {
      return false;
    }
    if (x == target_x && y == target_y) {
      return true;
    }
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      y += sy;
    }
  }
}

std::vector<RRTGlobalPlanner::Node> RRTGlobalPlanner::smoothPath(
    const std::vector<Node>& path) const {
  if (path.size() < 3) {
    return path;
  }

  std::vector<Node> smoothed;
  std::size_t current = 0;
  smoothed.push_back(path.front());
  while (current < path.size() - 1) {
    std::size_t next = path.size() - 1;
    while (next > current + 1 &&
           !segmentIsFree(path[current].x, path[current].y,
                          path[next].x, path[next].y)) {
      --next;
    }
    smoothed.push_back(path[next]);
    current = next;
  }
  return smoothed;
}

void RRTGlobalPlanner::publishPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) const {
  nav_msgs::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = ros::Time::now();
  path.poses = plan;
  plan_pub_.publish(path);
}

bool RRTGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  if (!initialized_ || costmap_ == nullptr) {
    ROS_ERROR("RRTGlobalPlanner is not initialized");
    return false;
  }
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    ROS_ERROR("RRT requires start and goal in %s", global_frame_.c_str());
    return false;
  }

  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    ROS_WARN("RRT start or goal lies outside the global costmap");
    return false;
  }
  if (!cellIsFree(start_x, start_y) || !cellIsFree(goal_x, goal_y)) {
    ROS_WARN("RRT start or goal is occupied");
    return false;
  }

  const double resolution = costmap_->getResolution();
  const double step_cells = std::max(1.0, step_size_ / resolution);
  const double tolerance_cells = std::max(1.0, goal_tolerance_ / resolution);
  std::uniform_int_distribution<unsigned int> sample_x(0, costmap_->getSizeInCellsX() - 1);
  std::uniform_int_distribution<unsigned int> sample_y(0, costmap_->getSizeInCellsY() - 1);
  std::uniform_real_distribution<double> probability(0.0, 1.0);

  std::vector<Node> tree;
  tree.reserve(static_cast<std::size_t>(max_iterations_) + 2);
  tree.push_back({start_x, start_y, -1});
  int goal_index = -1;

  for (int iteration = 0; iteration < max_iterations_; ++iteration) {
    unsigned int random_x = goal_x;
    unsigned int random_y = goal_y;
    if (probability(rng_) > goal_bias_) {
      random_x = sample_x(rng_);
      random_y = sample_y(rng_);
      if (!cellIsFree(random_x, random_y)) {
        continue;
      }
    }

    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < tree.size(); ++i) {
      const double dx = static_cast<double>(random_x) - tree[i].x;
      const double dy = static_cast<double>(random_y) - tree[i].y;
      const double distance = dx * dx + dy * dy;
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = i;
      }
    }

    const double dx = static_cast<double>(random_x) - tree[nearest].x;
    const double dy = static_cast<double>(random_y) - tree[nearest].y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1.0) {
      continue;
    }

    const double scale = std::min(step_cells, distance) / distance;
    const int candidate_x = static_cast<int>(std::lround(tree[nearest].x + dx * scale));
    const int candidate_y = static_cast<int>(std::lround(tree[nearest].y + dy * scale));
    if (candidate_x < 0 || candidate_y < 0) {
      continue;
    }
    const auto new_x = static_cast<unsigned int>(candidate_x);
    const auto new_y = static_cast<unsigned int>(candidate_y);
    if (!cellIsFree(new_x, new_y) ||
        !segmentIsFree(tree[nearest].x, tree[nearest].y, new_x, new_y)) {
      continue;
    }

    tree.push_back({new_x, new_y, static_cast<int>(nearest)});
    const double goal_distance = std::hypot(
        static_cast<double>(goal_x) - new_x,
        static_cast<double>(goal_y) - new_y);
    if (goal_distance <= tolerance_cells && segmentIsFree(new_x, new_y, goal_x, goal_y)) {
      tree.push_back({goal_x, goal_y, static_cast<int>(tree.size() - 1)});
      goal_index = static_cast<int>(tree.size() - 1);
      break;
    }
  }

  if (goal_index < 0) {
    ROS_WARN("RRT failed after %d iterations", max_iterations_);
    return false;
  }

  std::vector<Node> reverse_path;
  for (int index = goal_index; index >= 0; index = tree[static_cast<std::size_t>(index)].parent) {
    reverse_path.push_back(tree[static_cast<std::size_t>(index)]);
  }
  std::reverse(reverse_path.begin(), reverse_path.end());
  const std::vector<Node> cells = smoothPath(reverse_path);

  const ros::Time stamp = ros::Time::now();
  for (std::size_t i = 0; i < cells.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = stamp;
    costmap_->mapToWorld(cells[i].x, cells[i].y,
                         pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i + 1 < cells.size()) {
      double next_x = 0.0;
      double next_y = 0.0;
      costmap_->mapToWorld(cells[i + 1].x, cells[i + 1].y, next_x, next_y);
      yaw = std::atan2(next_y - pose.pose.position.y,
                       next_x - pose.pose.position.x);
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(quaternion);
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }
    plan.push_back(pose);
  }

  lock.unlock();
  publishPlan(plan);
  ROS_INFO("RRT produced a %zu-pose path from %zu sampled nodes",
           plan.size(), tree.size());
  return true;
}

}  // namespace autonomic_turtlebot3

PLUGINLIB_EXPORT_CLASS(autonomic_turtlebot3::RRTGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

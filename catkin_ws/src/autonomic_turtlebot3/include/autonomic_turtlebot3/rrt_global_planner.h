#ifndef AUTONOMIC_TURTLEBOT3_RRT_GLOBAL_PLANNER_H
#define AUTONOMIC_TURTLEBOT3_RRT_GLOBAL_PLANNER_H

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <random>
#include <string>
#include <vector>

namespace autonomic_turtlebot3 {

class RRTGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  RRTGlobalPlanner();
  RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  struct Node {
    unsigned int x;
    unsigned int y;
    int parent;
  };

  bool cellIsFree(unsigned int x, unsigned int y) const;
  bool segmentIsFree(unsigned int x0, unsigned int y0,
                     unsigned int x1, unsigned int y1) const;
  std::vector<Node> smoothPath(const std::vector<Node>& path) const;
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) const;

  bool initialized_;
  bool allow_unknown_;
  unsigned char lethal_cost_;
  double goal_bias_;
  double goal_tolerance_;
  double step_size_;
  int max_iterations_;
  std::string global_frame_;
  costmap_2d::Costmap2D* costmap_;
  ros::Publisher plan_pub_;
  mutable std::mt19937 rng_;
};

}  // namespace autonomic_turtlebot3

#endif  // AUTONOMIC_TURTLEBOT3_RRT_GLOBAL_PLANNER_H

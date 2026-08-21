// Copyright 2026 Assessment Student
// SPDX-License-Identifier: Apache-2.0

#ifndef TB3_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define TB3_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace tb3_astar_planner
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct QueueNode
  {
    std::size_t index;
    double f_score;

    bool operator>(const QueueNode & other) const
    {
      return f_score > other.f_score;
    }
  };

  bool isTraversable(unsigned int mx, unsigned int my) const;
  bool diagonalMoveIsSafe(
    unsigned int x, unsigned int y, int dx, int dy) const;
  double heuristic(
    unsigned int x, unsigned int y,
    unsigned int goal_x, unsigned int goal_y,
    bool accept_tolerance) const;
  double traversalCost(
    unsigned int from_x, unsigned int from_y,
    unsigned int to_x, unsigned int to_y) const;
  nav_msgs::msg::Path reconstructPath(
    std::size_t start_index,
    std::size_t reached_index,
    const std::vector<std::size_t> & parents,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    bool reached_exact_goal) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("tb3_astar_planner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string global_frame_;

  bool allow_unknown_{false};
  bool use_eight_connected_{true};
  double tolerance_{0.25};
  double cost_penalty_{2.0};
  int max_iterations_{1000000};
  unsigned char lethal_cost_{253};
};

}  // namespace tb3_astar_planner

#endif  // TB3_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

// Copyright 2026 Assessment Student
// SPDX-License-Identifier: Apache-2.0

#include "tb3_astar_planner/astar_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tb3_astar_planner
{

namespace
{
constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();

constexpr std::array<std::pair<int, int>, 8> kMoves{{
  {-1, 0}, {1, 0}, {0, -1}, {0, 1},
  {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
}};
}  // namespace

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("AStarPlanner parent node expired");
  }
  if (!costmap_ros || !costmap_ros->getCostmap()) {
    throw nav2_core::PlannerException("AStarPlanner received no costmap");
  }

  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_eight_connected", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_penalty", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(1000000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lethal_cost", rclcpp::ParameterValue(253));

  int lethal_cost_parameter = 253;
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".use_eight_connected", use_eight_connected_);
  node->get_parameter(name_ + ".tolerance", tolerance_);
  node->get_parameter(name_ + ".cost_penalty", cost_penalty_);
  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".lethal_cost", lethal_cost_parameter);

  if (tolerance_ < 0.0 || cost_penalty_ < 0.0 || max_iterations_ <= 0 ||
    lethal_cost_parameter < 1 || lethal_cost_parameter > 255)
  {
    throw nav2_core::PlannerException("AStarPlanner parameters are invalid");
  }
  lethal_cost_ = static_cast<unsigned char>(lethal_cost_parameter);

  RCLCPP_INFO(
    logger_,
    "Configured %s: %s-connected, tolerance=%.2f m, cost_penalty=%.2f",
    name_.c_str(), use_eight_connected_ ? "8" : "4", tolerance_, cost_penalty_);
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  clock_.reset();
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

bool AStarPlanner::isTraversable(unsigned int mx, unsigned int my) const
{
  const unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < lethal_cost_;
}

bool AStarPlanner::diagonalMoveIsSafe(
  unsigned int x, unsigned int y, int dx, int dy) const
{
  if (dx == 0 || dy == 0) {
    return true;
  }
  const int side_x = static_cast<int>(x) + dx;
  const int side_y = static_cast<int>(y) + dy;
  if (side_x < 0 || side_y < 0) {
    return false;
  }
  return isTraversable(static_cast<unsigned int>(side_x), y) &&
         isTraversable(x, static_cast<unsigned int>(side_y));
}

double AStarPlanner::heuristic(
  unsigned int x, unsigned int y,
  unsigned int goal_x, unsigned int goal_y,
  bool accept_tolerance) const
{
  const double dx = static_cast<double>(x) - static_cast<double>(goal_x);
  const double dy = static_cast<double>(y) - static_cast<double>(goal_y);
  double distance = std::hypot(dx, dy) * costmap_->getResolution();
  if (accept_tolerance) {
    distance = std::max(0.0, distance - tolerance_);
  }
  return distance;
}

double AStarPlanner::traversalCost(
  unsigned int from_x, unsigned int from_y,
  unsigned int to_x, unsigned int to_y) const
{
  const double step = std::hypot(
    static_cast<double>(to_x) - static_cast<double>(from_x),
    static_cast<double>(to_y) - static_cast<double>(from_y)) *
    costmap_->getResolution();
  const unsigned char raw_cost = costmap_->getCost(to_x, to_y);
  const double normalized_cost =
    raw_cost == nav2_costmap_2d::NO_INFORMATION ? 0.0 :
    static_cast<double>(raw_cost) /
    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
  return step * (1.0 + cost_penalty_ * normalized_cost);
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!costmap_ || !clock_) {
    throw nav2_core::PlannerException("AStarPlanner is not configured");
  }
  if ((!start.header.frame_id.empty() && start.header.frame_id != global_frame_) ||
    (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame_))
  {
    throw nav2_core::PlannerException(
            "Start and goal must use the global costmap frame " + global_frame_);
  }
  if (!std::isfinite(start.pose.position.x) ||
    !std::isfinite(start.pose.position.y) ||
    !std::isfinite(goal.pose.position.x) ||
    !std::isfinite(goal.pose.position.y))
  {
    throw nav2_core::PlannerException("Start or goal contains non-finite coordinates");
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *(costmap_->getMutex()));

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, start_x, start_y))
  {
    throw nav2_core::PlannerException("A* start lies outside the costmap");
  }
  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    throw nav2_core::PlannerException("A* goal lies outside the costmap");
  }
  if (!isTraversable(start_x, start_y)) {
    throw nav2_core::PlannerException(
            "A* start lies in an occupied costmap cell");
  }

  const bool exact_goal_traversable = isTraversable(goal_x, goal_y);
  if (!exact_goal_traversable && tolerance_ <= 0.0) {
    throw nav2_core::PlannerException(
            "A* goal lies in an occupied costmap cell");
  }

  const unsigned int width = costmap_->getSizeInCellsX();
  const unsigned int height = costmap_->getSizeInCellsY();
  const std::size_t cell_count = static_cast<std::size_t>(width) * height;
  const std::size_t start_index = costmap_->getIndex(start_x, start_y);

  std::vector<double> g_score(
    cell_count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parents(cell_count, kNoParent);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<
    QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

  g_score[start_index] = 0.0;
  open.push({start_index, heuristic(
      start_x, start_y, goal_x, goal_y, !exact_goal_traversable)});

  std::size_t reached_index = kNoParent;
  int iterations = 0;
  while (!open.empty() && iterations++ < max_iterations_) {
    const QueueNode current = open.top();
    open.pop();
    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = true;

    unsigned int current_x = 0;
    unsigned int current_y = 0;
    costmap_->indexToCells(current.index, current_x, current_y);
    const double distance_to_goal = std::hypot(
      static_cast<double>(current_x) - static_cast<double>(goal_x),
      static_cast<double>(current_y) - static_cast<double>(goal_y)) *
      costmap_->getResolution();
    const bool reached = exact_goal_traversable ?
      (current_x == goal_x && current_y == goal_y) :
      (distance_to_goal <= tolerance_);
    if (reached) {
      reached_index = current.index;
      break;
    }

    const std::size_t move_count = use_eight_connected_ ? 8U : 4U;
    for (std::size_t move_index = 0; move_index < move_count; ++move_index) {
      const int dx = kMoves[move_index].first;
      const int dy = kMoves[move_index].second;
      const int next_x_signed = static_cast<int>(current_x) + dx;
      const int next_y_signed = static_cast<int>(current_y) + dy;
      if (next_x_signed < 0 || next_y_signed < 0 ||
        next_x_signed >= static_cast<int>(width) ||
        next_y_signed >= static_cast<int>(height))
      {
        continue;
      }

      const auto next_x = static_cast<unsigned int>(next_x_signed);
      const auto next_y = static_cast<unsigned int>(next_y_signed);
      if (!isTraversable(next_x, next_y) ||
        !diagonalMoveIsSafe(current_x, current_y, dx, dy))
      {
        continue;
      }

      const std::size_t next_index = costmap_->getIndex(next_x, next_y);
      if (closed[next_index]) {
        continue;
      }
      const double tentative_g = g_score[current.index] +
        traversalCost(current_x, current_y, next_x, next_y);
      if (tentative_g >= g_score[next_index]) {
        continue;
      }

      parents[next_index] = current.index;
      g_score[next_index] = tentative_g;
      open.push({
        next_index,
        tentative_g + heuristic(
          next_x, next_y, goal_x, goal_y, !exact_goal_traversable)
      });
    }
  }

  if (reached_index == kNoParent) {
    throw nav2_core::PlannerException(
            "A* exhausted its search without finding a path");
  }

  RCLCPP_DEBUG(
    logger_, "A* found a path after %d iterations", iterations);
  return reconstructPath(
    start_index, reached_index, parents, start, goal,
    reached_index == costmap_->getIndex(goal_x, goal_y));
}

nav_msgs::msg::Path AStarPlanner::reconstructPath(
  std::size_t start_index,
  std::size_t reached_index,
  const std::vector<std::size_t> & parents,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool reached_exact_goal) const
{
  std::vector<std::size_t> indices;
  for (std::size_t index = reached_index; index != kNoParent;
    index = parents[index])
  {
    indices.push_back(index);
    if (index == start_index) {
      break;
    }
  }
  if (indices.empty() || indices.back() != start_index) {
    throw nav2_core::PlannerException("A* parent chain is incomplete");
  }
  std::reverse(indices.begin(), indices.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  path.poses.reserve(indices.size());

  for (const std::size_t index : indices) {
    unsigned int mx = 0;
    unsigned int my = 0;
    costmap_->indexToCells(index, mx, my);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    costmap_->mapToWorld(mx, my, pose.pose.position.x, pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  if (!path.poses.empty()) {
    path.poses.front().pose.position = start.pose.position;
  }

  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const double yaw = std::atan2(
      path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y,
      path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x);
    path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
    path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
  }

  if (reached_exact_goal && !path.poses.empty()) {
    path.poses.back().pose = goal.pose;
  } else if (!path.poses.empty()) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }
  return path;
}

}  // namespace tb3_astar_planner

PLUGINLIB_EXPORT_CLASS(tb3_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)

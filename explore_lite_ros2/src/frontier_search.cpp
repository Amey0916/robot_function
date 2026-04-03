#include "explore_lite_ros2/frontier_search.hpp"
#include "explore_lite_ros2/costmap_tools.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <iostream>

#include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace explore_lite
{
FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale,
                               double gain_scale,
                               double min_frontier_size,
                               double max_frontier_distance)  // 新增参数
  : costmap_(costmap),
    potential_scale_(potential_scale),
    gain_scale_(gain_scale),
    min_frontier_size_(min_frontier_size),
    max_frontier_distance_(max_frontier_distance)  // 初始化新增参数
{
}

std::vector<Frontier> FrontierSearch::searchFrom(const geometry_msgs::msg::Point& position)
{
  std::vector<Frontier> frontier_list;

  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(rclcpp::get_logger("explore_lite"),
                 "Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  std::queue<unsigned int> bfs;

  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, nav2_costmap_2d::FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(rclcpp::get_logger("explore_lite"),
                "Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    for (unsigned nbr : nhood4(idx, *costmap_)) {
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        // 过滤条件1：最小尺寸
        if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
          // 过滤条件2：最大距离
          if (new_frontier.min_distance <= max_frontier_distance_) {
            frontier_list.push_back(new_frontier);
          } else {
            RCLCPP_DEBUG(rclcpp::get_logger("explore_lite"),
                         "Frontier filtered: distance %.2fm > max %.2fm",
                         new_frontier.min_distance, max_frontier_distance_);
          }
        }
      }
    }
  }

  // 过滤不可达的前沿
  std::vector<Frontier> reachable_frontiers;
  for (auto& frontier : frontier_list) {
    if (isFrontierReachable(position, frontier)) {
      reachable_frontiers.push_back(frontier);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("explore_lite"),
                  "Frontier at (%.2f, %.2f) is unreachable (obstacle blocked)",
                  frontier.centroid.x, frontier.centroid.y);
    }
  }

  // 重新计算代价并排序
  for (auto& frontier : reachable_frontiers) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(reachable_frontiers.begin(), reachable_frontiers.end(),
            [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return reachable_frontiers;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  Frontier output;
  output.centroid.x = 0.0;
  output.centroid.y = 0.0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;

        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        output.size++;
        output.centroid.x += wx;
        output.centroid.y += wy;

        double distance = std::hypot(reference_x - wx, reference_y - wy);
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        bfs.push(nbr);
      }
    }
  }

  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  if (map_[idx] != nav2_costmap_2d::NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == nav2_costmap_2d::FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance * costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}

// 新增：校验前沿可达性
bool FrontierSearch::isFrontierReachable(const geometry_msgs::msg::Point& robot_pose, 
                                         const Frontier& frontier)
{
  unsigned int robot_mx, robot_my;
  unsigned int frontier_mx, frontier_my;

  // 转换机器人位置到栅格
  if (!costmap_->worldToMap(robot_pose.x, robot_pose.y, robot_mx, robot_my)) {
    return false;
  }
  // 转换前沿中心点到栅格
  if (!costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, frontier_mx, frontier_my)) {
    return false;
  }

  unsigned int robot_idx = costmap_->getIndex(robot_mx, robot_my);
  unsigned int frontier_idx = costmap_->getIndex(frontier_mx, frontier_my);

  return isCellReachable(robot_idx, frontier_idx);
}

// 新增：BFS检查两个栅格之间的可达性（仅通过自由空间）
bool FrontierSearch::isCellReachable(unsigned int start_idx, unsigned int target_idx)
{
  if (start_idx == target_idx) return true;
  if (map_[target_idx] != nav2_costmap_2d::NO_INFORMATION && map_[target_idx] != nav2_costmap_2d::FREE_SPACE) {
    return false;
  }

  std::vector<bool> visited(size_x_ * size_y_, false);
  std::queue<unsigned int> bfs;

  bfs.push(start_idx);
  visited[start_idx] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    for (unsigned int nbr : nhood4(idx, *costmap_)) {
      if (nbr == target_idx) return true;
      if (!visited[nbr] && map_[nbr] == nav2_costmap_2d::FREE_SPACE) {
        visited[nbr] = true;
        bfs.push(nbr);
      }
    }
  }

  return false;
}
}  // namespace explore_lite

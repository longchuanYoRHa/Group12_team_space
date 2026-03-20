#ifndef CUSTOM_EXPLORE_COSTMAP_TOOLS_H_
#define CUSTOM_EXPLORE_COSTMAP_TOOLS_H_

#include <queue>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace frontier_exploration
{
/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map
 * edges.
 */
std::vector<unsigned int> nhood4(unsigned int idx,
                                 const nav2_costmap_2d::Costmap2D& costmap)
{
  std::vector<unsigned int> out;

  unsigned int size_x_ = costmap.getSizeInCellsX();
  unsigned int size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    RCLCPP_WARN(rclcpp::get_logger("FrontierExploration"), "Evaluating nhood "
                                                           "for offmap point");
    return out;
  }

  if (idx % size_x_ > 0) {
    out.push_back(idx - 1);
  }
  if (idx % size_x_ < size_x_ - 1) {
    out.push_back(idx + 1);
  }
  if (idx >= size_x_) {
    out.push_back(idx - size_x_);
  }
  if (idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + size_x_);
  }
  return out;
}

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map
 * edges.
 */
std::vector<unsigned int> nhood8(unsigned int idx,
                                 const nav2_costmap_2d::Costmap2D& costmap)
{
  std::vector<unsigned int> out = nhood4(idx, costmap);

  unsigned int size_x_ = costmap.getSizeInCellsX();
  unsigned int size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    return out;
  }

  if (idx % size_x_ > 0 && idx >= size_x_) {
    out.push_back(idx - 1 - size_x_);
  }
  if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx - 1 + size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
    out.push_back(idx + 1 - size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + 1 + size_x_);
  }

  return out;
}

/**
 * @brief Find nearest cell of a specified value.
 */
bool nearestCell(unsigned int& result, unsigned int start, unsigned char val,
                  const nav2_costmap_2d::Costmap2D& costmap)
{
  const unsigned char* map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  if (start >= size_x * size_y) {
    return false;
  }

  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  bfs.push(start);
  visited_flag[start] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    if (map[idx] == val) {
      result = idx;
      return true;
    }

    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
      }
    }
  }

  return false;
}
}  // namespace frontier_exploration

#endif


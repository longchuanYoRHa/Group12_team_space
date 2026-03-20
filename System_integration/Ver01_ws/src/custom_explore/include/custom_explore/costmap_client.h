#ifndef CUSTOM_EXPLORE_COSTMAP_CLIENT_H_
#define CUSTOM_EXPLORE_COSTMAP_CLIENT_H_

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>

#include <rclcpp/rclcpp.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace explore
{
class Costmap2DClient
{
public:
  Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf);

  geometry_msgs::msg::Pose getRobotPose() const;

  nav2_costmap_2d::Costmap2D* getCostmap()
  {
    return &costmap_;
  }

  const nav2_costmap_2d::Costmap2D* getCostmap() const
  {
    return &costmap_;
  }

  const std::string& getGlobalFrameID() const
  {
    return global_frame_;
  }

  const std::string& getBaseFrameID() const
  {
    return robot_base_frame_;
  }

protected:
  void updateFullMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void updatePartialMap(
      const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);

  nav2_costmap_2d::Costmap2D costmap_;
  bool costmap_received_ = false;

  const tf2_ros::Buffer* const tf_;
  rclcpp::Node& node_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      costmap_updates_sub_;
};
}  // namespace explore

#endif


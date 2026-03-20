/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *
 *********************************************************************/

#include <custom_explore/costmap_client.h>

#include <unistd.h>

#include <array>
#include <functional>
#include <mutex>
#include <string>

namespace explore
{
// Static translation table to speed things up.
std::array<unsigned char, 256> init_translation_table();
static const std::array<unsigned char, 256> cost_translation_table__ =
    init_translation_table();

Costmap2DClient::Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf)
  : tf_(tf), node_(node)
{
  std::string costmap_topic;
  std::string costmap_updates_topic;

  node_.declare_parameter<std::string>("costmap_topic", std::string("costmap"));
  node_.declare_parameter<std::string>("costmap_updates_topic",
                                       std::string("costmap_updates"));
  node_.declare_parameter<std::string>("robot_base_frame", std::string("base_link"));
  node_.declare_parameter<double>("transform_tolerance", 0.3);

  node_.get_parameter("costmap_topic", costmap_topic);
  node_.get_parameter("costmap_updates_topic", costmap_updates_topic);
  node_.get_parameter("robot_base_frame", robot_base_frame_);
  node_.get_parameter("transform_tolerance", transform_tolerance_);

  // Initialize costmap.
  costmap_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, 1000,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_received_ = true;
        updateFullMap(msg);
      });

  // Wait for the first costmap message.
  RCLCPP_INFO(node_.get_logger(),
              "Waiting for costmap to become available, topic: %s",
              costmap_topic.c_str());
  while (!costmap_received_) {
    rclcpp::spin_some(node_.get_node_base_interface());
    usleep(1000000);
  }

  // Subscribe to map updates.
  costmap_updates_sub_ =
      node_.create_subscription<map_msgs::msg::OccupancyGridUpdate>(
          costmap_updates_topic, 1000,
          [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
            updatePartialMap(msg);
          });

  // Make sure tf transform from robot base to global frame exists.
  auto last_error = node_.now();
  std::string tf_error;
  while (rclcpp::ok() &&
         !tf_->canTransform(global_frame_, robot_base_frame_,
                            tf2::TimePointZero,
                            tf2::durationFromSec(0.1), &tf_error)) {
    rclcpp::spin_some(node_.get_node_base_interface());
    if (last_error + tf2::durationFromSec(5.0) < node_.now()) {
      RCLCPP_WARN(node_.get_logger(),
                  "Timed out waiting for transform from %s to %s to become "
                  "available before subscribing to costmap, tf error: %s",
                  robot_base_frame_.c_str(), global_frame_.c_str(),
                  tf_error.c_str());
      last_error = node_.now();
    }
    tf_error.clear();
  }
}

void Costmap2DClient::updateFullMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  global_frame_ = msg->header.frame_id;

  unsigned int size_in_cells_x = msg->info.width;
  unsigned int size_in_cells_y = msg->info.height;
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;

  RCLCPP_DEBUG(node_.get_logger(), "received full new map, resizing to: %d, %d",
               size_in_cells_x, size_in_cells_y);
  costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x,
                      origin_y);

  // Lock as we access raw underlying map.
  auto* mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  // Fill map with data.
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
  RCLCPP_DEBUG(node_.get_logger(), "full map update, %lu values", costmap_size);
  for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
    unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
    costmap_data[i] = cost_translation_table__[cell_cost];
  }
  RCLCPP_DEBUG(node_.get_logger(), "map updated, written %lu values", costmap_size);
}

void Costmap2DClient::updatePartialMap(
    const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
{
  RCLCPP_DEBUG(node_.get_logger(), "received partial map update");
  global_frame_ = msg->header.frame_id;

  if (msg->x < 0 || msg->y < 0) {
    RCLCPP_DEBUG(node_.get_logger(),
                 "negative coordinates, invalid update. x: %d, y: %d", msg->x,
                 msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  auto* mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  size_t costmap_xn = costmap_.getSizeInCellsX();
  size_t costmap_yn = costmap_.getSizeInCellsY();

  if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
      y0 > costmap_yn) {
    RCLCPP_WARN(node_.get_logger(),
                "received update doesn't fully fit into existing map, "
                "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
                "map is: [0, %lu], [0, %lu]",
                x0, xn, y0, yn, costmap_xn, costmap_yn);
  }

  unsigned char* costmap_data = costmap_.getCharMap();
  size_t i = 0;
  for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
    for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
      size_t idx = costmap_.getIndex(x, y);
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[idx] = cost_translation_table__[cell_cost];
      ++i;
    }
  }
}

geometry_msgs::msg::Pose Costmap2DClient::getRobotPose() const
{
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Pose empty_pose;
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = node_.now();

  auto& clk = *node_.get_clock();
  try {
    robot_pose =
        tf_->transform(robot_pose, global_frame_,
                       tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "No Transform available Error looking up robot pose: %s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "Connectivity Error looking up robot pose: %s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "Extrapolation Error looking up robot pose: %s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000, "Other error: %s\n",
                          ex.what());
    return empty_pose;
  }

  return robot_pose.pose;
}

std::array<unsigned char, 256> init_translation_table()
{
  std::array<unsigned char, 256> cost_translation_table;

  // Linearly map [0..100] to [0..255].
  for (size_t i = 0; i < 256; ++i) {
    cost_translation_table[i] =
        static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  // Special values.
  cost_translation_table[0] = 0;      // NO obstacle
  cost_translation_table[99] = 253;   // INSCRIBED obstacle
  cost_translation_table[100] = 254;  // LETHAL obstacle
  cost_translation_table[static_cast<unsigned char>(-1)] = 255;  // UNKNOWN

  return cost_translation_table;
}

}  // namespace explore


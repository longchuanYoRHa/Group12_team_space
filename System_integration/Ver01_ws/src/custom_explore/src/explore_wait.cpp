/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2008, Robert Bosch LLC.
 * Copyright (c) 2015-2016, Jiri Horner.
 * Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *
 *********************************************************************/

#include <custom_explore/costmap_client.h>
#include <custom_explore/frontier_search.h>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"

// Keep the original explore_lite convention for action name depending on ROS distro.
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif

namespace explore
{
static constexpr int kBlacklistAbortThreshold = 3;

struct FrontierBlacklistEntry {
  geometry_msgs::msg::Point point;
  int abort_count{0};
};

class ExploreWait : public rclcpp::Node
{
public:
  ExploreWait()
    : Node("custom_explore_node")
    , logger_(this->get_logger())
    , tf_buffer_(this->get_clock())
    , tf_listener_(tf_buffer_)
    , costmap_client_(*this, &tf_buffer_)
    , prev_distance_(0.0)
    , last_markers_count_(0)
    , last_progress_(this->now())
  {
    double timeout = 0.0;
    double min_frontier_size = 0.5;

    // Params may already be declared by params.yaml and/or Costmap2DClient.
    if (!this->has_parameter("planner_frequency")) {
      this->declare_parameter<float>("planner_frequency", 1.0);
    }
    if (!this->has_parameter("progress_timeout")) {
      this->declare_parameter<float>("progress_timeout", 90.0);
    }
    if (!this->has_parameter("visualize")) {
      this->declare_parameter<bool>("visualize", false);
    }
    if (!this->has_parameter("potential_scale")) {
      this->declare_parameter<float>("potential_scale", 1e-3);
    }
    if (!this->has_parameter("orientation_scale")) {
      this->declare_parameter<float>("orientation_scale", 0.0);
    }
    if (!this->has_parameter("gain_scale")) {
      this->declare_parameter<float>("gain_scale", 1.0);
    }
    if (!this->has_parameter("min_frontier_size")) {
      this->declare_parameter<float>("min_frontier_size", 0.5);
    }
    if (!this->has_parameter("return_to_init")) {
      this->declare_parameter<bool>("return_to_init", false);
    }
    if (!this->has_parameter("robot_base_frame")) {
      this->declare_parameter<std::string>("robot_base_frame", "base_link");
    }

    this->get_parameter("planner_frequency", planner_frequency_);
    this->get_parameter("progress_timeout", timeout);
    this->get_parameter("visualize", visualize_);
    this->get_parameter("potential_scale", potential_scale_);
    this->get_parameter("orientation_scale", orientation_scale_);
    this->get_parameter("gain_scale", gain_scale_);
    this->get_parameter("min_frontier_size", min_frontier_size);
    this->get_parameter("return_to_init", return_to_init_);
    this->get_parameter("robot_base_frame", robot_base_frame_);

    progress_timeout_ = timeout;

    move_base_client_ = rclcpp_action::create_client<
        nav2_msgs::action::NavigateToPose>(this, ACTION_NAME);

    search_ = frontier_exploration::FrontierSearch(
        costmap_client_.getCostmap(), potential_scale_, gain_scale_,
        min_frontier_size, logger_);

    if (visualize_) {
      marker_array_publisher_ = this->create_publisher<
          visualization_msgs::msg::MarkerArray>("explore/frontiers", 10);
    }

    // Control start/stop of exploration behavior via /explore/resume topic.
    // Default is waiting (inactive) until we receive explore/resume=true.
    resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "explore/resume", 10,
        std::bind(&ExploreWait::resumeCallback, this,
                  std::placeholders::_1));

    exploration_finished_pub_ =
        this->create_publisher<std_msgs::msg::Bool>("explore/finished", 10);

    RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
    move_base_client_->wait_for_action_server();
    RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

    if (return_to_init_) {
      RCLCPP_INFO(logger_, "Getting initial pose of the robot");
      geometry_msgs::msg::TransformStamped transform_stamped;
      std::string map_frame = costmap_client_.getGlobalFrameID();
      try {
        transform_stamped = tf_buffer_.lookupTransform(
            map_frame, robot_base_frame_, tf2::TimePointZero);
        initial_pose_.position.x = transform_stamped.transform.translation.x;
        initial_pose_.position.y = transform_stamped.transform.translation.y;
        initial_pose_.orientation = transform_stamped.transform.rotation;
      } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(logger_,
                     "Couldn't find transform from %s to %s: %s",
                     map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
        return_to_init_ = false;
      }
    }

    exploring_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<uint16_t>(
            1000.0 / std::max(planner_frequency_, 0.001))),
        [this]() {
          if (!active_) {
            return;
          }
          // Frontier search only runs in makePlan(); timer only watches progress
          // toward the current NavigateToPose goal.
          if (nav_goal_in_flight_) {
            checkNavProgress();
          }
        });

    // Default waiting state: never run makePlan() until explore/resume=true.
    active_ = false;
    resuming_ = false;
    exploring_timer_->cancel();
  }

  ~ExploreWait() override { stop(false); }

private:
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      resume();
    } else {
      stop(false);
    }
  }

  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers)
  {
    if (!visualize_ || !marker_array_publisher_) {
      return;
    }

    std_msgs::msg::ColorRGBA blue;
    blue.r = 0;
    blue.g = 0;
    blue.b = 1.0;
    blue.a = 1.0;
    std_msgs::msg::ColorRGBA red;
    red.r = 1.0;
    red.g = 0;
    red.b = 0;
    red.a = 1.0;
    std_msgs::msg::ColorRGBA green;
    green.r = 0;
    green.g = 1.0;
    green.b = 0;
    green.a = 1.0;

    RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
    visualization_msgs::msg::MarkerArray markers_msg;
    std::vector<visualization_msgs::msg::Marker>& markers =
        markers_msg.markers;
    visualization_msgs::msg::Marker m;

    m.header.frame_id = costmap_client_.getGlobalFrameID();
    m.header.stamp = this->now();
    m.ns = "frontiers";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;

    // Marker lifetime: lives forever.
    // Keep same ELOQUENT/DASHING compatibility as explore_lite.
#ifdef ELOQUENT
    m.lifetime = rclcpp::Duration(0);
#elif DASHING
    m.lifetime = rclcpp::Duration(0);
#else
    m.lifetime = rclcpp::Duration::from_seconds(0);
#endif
    m.frame_locked = true;

    double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

    m.action = visualization_msgs::msg::Marker::ADD;
    size_t id = 0;
    for (auto& frontier : frontiers) {
      m.type = visualization_msgs::msg::Marker::POINTS;
      m.id = static_cast<int>(id);
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      if (goalOnBlacklist(frontier.centroid)) {
        m.color = red;
      } else {
        m.color = blue;
      }
      markers.push_back(m);
      ++id;

      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.id = static_cast<int>(id);
      m.pose.position = frontier.initial;
      // Scale frontier according to its cost (costier frontiers will be smaller).
      double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      m.points = {};
      m.color = green;
      markers.push_back(m);
      ++id;
    }

    size_t current_markers_count = markers.size();
    m.action = visualization_msgs::msg::Marker::DELETE;
    for (; id < last_markers_count_; ++id) {
      m.id = static_cast<int>(id);
      markers.push_back(m);
    }
    last_markers_count_ = current_markers_count;
    marker_array_publisher_->publish(markers_msg);
  }

  std::vector<FrontierBlacklistEntry>::iterator findBlacklistEntry(
      const geometry_msgs::msg::Point& goal)
  {
    constexpr static size_t tolerance_cells = 5;
    auto* costmap2d = costmap_client_.getCostmap();
    const double tol = static_cast<double>(tolerance_cells) * costmap2d->getResolution();
    for (auto it = frontier_blacklist_.begin(); it != frontier_blacklist_.end();
         ++it) {
      const double x_diff = std::fabs(goal.x - it->point.x);
      const double y_diff = std::fabs(goal.y - it->point.y);
      if (x_diff < tol && y_diff < tol) {
        return it;
      }
    }
    return frontier_blacklist_.end();
  }

  void bumpAbortFailure(const geometry_msgs::msg::Point& p)
  {
    auto it = findBlacklistEntry(p);
    if (it != frontier_blacklist_.end()) {
      it->abort_count =
          std::min(it->abort_count + 1, kBlacklistAbortThreshold);
    } else {
      frontier_blacklist_.push_back({p, 1});
    }
  }

  void blacklistForProgressTimeout(const geometry_msgs::msg::Point& p)
  {
    auto it = findBlacklistEntry(p);
    if (it != frontier_blacklist_.end()) {
      it->abort_count =
          std::max(it->abort_count, kBlacklistAbortThreshold);
    } else {
      frontier_blacklist_.push_back({p, kBlacklistAbortThreshold});
    }
  }

  void checkNavProgress()
  {
    if (!nav_goal_in_flight_) {
      return;
    }

    auto pose = costmap_client_.getRobotPose();
    const double dx = pose.position.x - current_nav_target_.x;
    const double dy = pose.position.y - current_nav_target_.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < prev_distance_) {
      last_progress_ = this->now();
      prev_distance_ = dist;
    }

    if (resuming_) {
      return;
    }

    if (this->now() - last_progress_ >
        tf2::durationFromSec(progress_timeout_)) {
      RCLCPP_DEBUG(logger_,
                   "Progress timeout toward current goal; blacklisting and "
                   "replanning.");
      blacklistForProgressTimeout(current_nav_target_);
      nav_goal_in_flight_ = false;
      if (active_) {
        makePlan();
      }
    }
  }

  void makePlan()
  {
    if (!active_) {
      return;
    }

    auto pose = costmap_client_.getRobotPose();
    auto frontiers = search_.searchFrom(pose.position);
    RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
    if (frontiers.empty()) {
      RCLCPP_WARN(logger_, "No frontiers found, stopping.");
      stop(true);
      return;
    }

    if (visualize_) {
      visualizeFrontiers(frontiers);
    }

    // find non blacklisted frontier
    auto frontier = std::find_if_not(
        frontiers.begin(), frontiers.end(),
        [this](const frontier_exploration::Frontier& f) {
          return goalOnBlacklist(f.centroid);
        });
    if (frontier == frontiers.end()) {
      RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
      stop(true);
      return;
    }

    geometry_msgs::msg::Point target_position = frontier->centroid;

    RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

    current_nav_target_ = target_position;
    last_progress_ = this->now();
    const double tdx = pose.position.x - target_position.x;
    const double tdy = pose.position.y - target_position.y;
    prev_distance_ = std::sqrt(tdx * tdx + tdy * tdy);
    nav_goal_in_flight_ = true;
    ++nav_goal_generation_;
    const uint64_t goal_generation = nav_goal_generation_;

    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = target_position;
    goal.pose.pose.orientation.w = 1.;
    goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.pose.header.stamp = this->now();

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [this, target_position, goal_generation](
            const rclcpp_action::ClientGoalHandle<
                nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
          if (goal_generation != nav_goal_generation_) {
            return;
          }
          reachedGoal(result, target_position);
        };

    move_base_client_->async_send_goal(goal, send_goal_options);

    if (resuming_) {
      resuming_ = false;
    }
  }

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal)
  {
    auto it = findBlacklistEntry(goal);
    return it != frontier_blacklist_.end() &&
           it->abort_count >= kBlacklistAbortThreshold;
  }

  void reachedGoal(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::
          WrappedResult& result,
      const geometry_msgs::msg::Point& frontier_goal)
  {
    nav_goal_in_flight_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_DEBUG(logger_, "Goal was successful");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_DEBUG(logger_, "Goal was aborted");
        bumpAbortFailure(frontier_goal);
        if (active_) {
          makePlan();
        }
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_DEBUG(logger_, "Goal was canceled");
        // If goal canceled might be because exploration stopped from topic.
        return;
      default:
        RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
        break;
    }

    if (active_) {
      makePlan();
    }
  }

  void returnToInitialPose()
  {
    RCLCPP_INFO(logger_, "Returning to initial pose.");
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = initial_pose_.position;
    goal.pose.pose.orientation = initial_pose_.orientation;
    goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.pose.header.stamp = this->now();

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    move_base_client_->async_send_goal(goal, send_goal_options);
  }

  void stop(bool finished_exploring)
  {
    if (!active_ && !finished_exploring) {
      return;
    }

    RCLCPP_INFO(logger_, "Exploration stopped.");
    active_ = false;
    resuming_ = false;
    nav_goal_in_flight_ = false;
    ++nav_goal_generation_;
    move_base_client_->async_cancel_all_goals();
    exploring_timer_->cancel();

    if (finished_exploring) {
      std_msgs::msg::Bool msg;
      msg.data = true;
      exploration_finished_pub_->publish(msg);
      RCLCPP_INFO(logger_,
                  "Exploration finished signal published (explore/finished = true).");
    }

    if (return_to_init_ && finished_exploring) {
      returnToInitialPose();
    }
  }

  void resume()
  {
    if (active_) {
      return;
    }

    active_ = true;
    resuming_ = true;
    RCLCPP_INFO(logger_, "Exploration resuming.");

    exploring_timer_->reset();
    // Resume immediately.
    makePlan();
  }

  // --- Members ---
  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  explore::Costmap2DClient costmap_client_;
  frontier_exploration::FrontierSearch search_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;

  rclcpp::TimerBase::SharedPtr exploring_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_finished_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;

  std::vector<FrontierBlacklistEntry> frontier_blacklist_;
  geometry_msgs::msg::Point current_nav_target_;
  bool nav_goal_in_flight_{false};
  uint64_t nav_goal_generation_{0};
  double prev_distance_;
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  geometry_msgs::msg::Pose initial_pose_;
  bool active_{false};
  bool resuming_{false};

  // Parameters
  double planner_frequency_{1.0};
  double potential_scale_{1e-3}, orientation_scale_{0.0}, gain_scale_{1.0};
  double progress_timeout_{90.0};
  bool visualize_{false};
  bool return_to_init_{false};
  std::string robot_base_frame_;
};
}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explore::ExploreWait>());
  rclcpp::shutdown();
  return 0;
}


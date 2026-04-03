#include "explore_lite_ros2/explore_node.hpp"
#include "explore_lite_ros2/costmap_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace explore_lite
{
using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

ExploreNode::ExploreNode() : Node("explore_lite_node"), prev_distance_{0.0}
{
  // 修复：显式初始化prev_goal_的x/y/z
  prev_goal_.x = 0.0;
  prev_goal_.y = 0.0;
  prev_goal_.z = 0.0;

  // 声明参数
  this->declare_parameter("planner_frequency", 0.2);
  this->declare_parameter("progress_timeout", 60.0);
  this->declare_parameter("visualize", true);
  this->declare_parameter("potential_scale", 1e-3);
  this->declare_parameter("gain_scale", 1.0);
  this->declare_parameter("min_frontier_size", 0.5);
  this->declare_parameter("max_frontier_distance", 5.0);
  this->declare_parameter("costmap_topic", "global_costmap/costmap");
  this->declare_parameter("costmap_updates_topic", "global_costmap/costmap_updates");
  this->declare_parameter("robot_base_frame", "base_link");
  this->declare_parameter("transform_tolerance", 0.3);
  this->declare_parameter("action_server_name", "navigate_to_pose");
  this->declare_parameter("map_topic", "/map");

  // 获取参数
  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", progress_timeout_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size_);
  this->get_parameter("max_frontier_distance", max_frontier_distance_);
  this->get_parameter("action_server_name", action_server_name_);

  // 初始化TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化代价地图客户端
  CostmapClientOptions options;
  options.costmap_topic = this->get_parameter("costmap_topic").as_string();
  options.costmap_updates_topic = this->get_parameter("costmap_updates_topic").as_string();
  options.robot_base_frame = this->get_parameter("robot_base_frame").as_string();
  options.transform_tolerance = this->get_parameter("transform_tolerance").as_double();
  costmap_client_ = std::make_unique<Costmap2DClient>(*this, tf_buffer_, options);

  // 初始化前沿搜索
  search_ = FrontierSearch(
      &costmap_client_->getCostmap(),
      potential_scale_,
      gain_scale_,
      min_frontier_size_,
      max_frontier_distance_
  );

  // 初始化导航客户端
  nav_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
      this, action_server_name_);

  // 初始化可视化发布器
  if (visualize_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "explore/frontiers", rclcpp::QoS(1));
  }

  // 初始化探索定时器
  exploring_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / planner_frequency_),
      std::bind(&ExploreNode::makePlan, this));

  last_progress_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Explore node initialized, planner frequency: %.2f Hz", planner_frequency_);
}

void ExploreNode::makePlan()
{
  // 修复：ROS2 Humble用!is_canceled()判断定时器是否激活
  if (exploring_timer_->is_canceled()) {
    return;
  }

  // 获取机器人位姿
  auto robot_pose_opt = costmap_client_->getRobotPose();
  if (!robot_pose_opt) {
    RCLCPP_WARN(this->get_logger(), "Failed to get robot pose, skip planning");
    return;
  }
  geometry_msgs::msg::Pose robot_pose = *robot_pose_opt;

  // 搜索前沿
  std::vector<Frontier> frontiers = search_.searchFrom(robot_pose.position);

  // 判定是否全探索完成
  if (frontiers.empty()) {
    RCLCPP_INFO(this->get_logger(), "✅ All frontiers explored! Stopping exploration timer");
    exploring_timer_->cancel();
    visualizeFrontiers(frontiers);
    return;
  }

  // 过滤黑名单中的前沿
  std::vector<Frontier> valid_frontiers;
  for (auto& f : frontiers) {
    if (!goalOnBlacklist(f.centroid)) {
      valid_frontiers.push_back(f);
    }
  }

  // 再次检查有效前沿
  if (valid_frontiers.empty()) {
    RCLCPP_INFO(this->get_logger(), "✅ No valid frontiers left (all in blacklist)! Exploration completed");
    exploring_timer_->cancel();
    visualizeFrontiers(valid_frontiers);
    return;
  }

  // 选择最优前沿
  Frontier best_frontier = valid_frontiers[0];

  // 检查进度超时
  double distance = std::hypot(
      robot_pose.position.x - prev_goal_.x,
      robot_pose.position.y - prev_goal_.y);
  if (distance > 0.1) {
    last_progress_ = this->now();
    prev_goal_ = best_frontier.centroid;
    prev_distance_ = 0.0;
  } else {
    prev_distance_ += distance;
    // 将double秒转换成Duration比较
    if ((this->now() - last_progress_) > rclcpp::Duration::from_seconds(progress_timeout_)) {
      RCLCPP_WARN(this->get_logger(), "Progress timeout, adding frontier to blacklist");
      frontier_blacklist_.push_back(best_frontier.centroid);
      return;
    }
  }

  // 发布导航目标
  auto goal_msg = NavigateToPoseAction::Goal();
  goal_msg.pose.header.frame_id = costmap_client_->getGlobalFrame();
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position = best_frontier.centroid;
  goal_msg.pose.pose.orientation.w = 1.0;

  // 发送目标
  auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&ExploreNode::reachedGoal, this, std::placeholders::_1);
  nav_client_->async_send_goal(goal_msg, send_goal_options);

  // 可视化前沿
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  RCLCPP_INFO(this->get_logger(), "Sending exploration goal to (%.2f, %.2f)",
              best_frontier.centroid.x, best_frontier.centroid.y);
}

void ExploreNode::visualizeFrontiers(const std::vector<Frontier>& frontiers)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_marker);

  if (frontiers.empty()) {
    marker_pub_->publish(markers);
    return;
  }

  int id = 0;
  for (const auto& f : frontiers) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap_client_->getGlobalFrame();
    marker.header.stamp = this->now();
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = f.centroid;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0 / planner_frequency_);
    markers.markers.push_back(marker);
  }

  marker_pub_->publish(markers);
}

void ExploreNode::reachedGoal(const ClientGoalHandle::WrappedResult& result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Reached exploration goal");
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Failed to reach exploration goal, adding to blacklist");
      frontier_blacklist_.push_back(prev_goal_);
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown result code: %d", static_cast<int>(result.code));
      frontier_blacklist_.push_back(prev_goal_);
      break;
  }
}

bool ExploreNode::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  for (const auto& bl_goal : frontier_blacklist_) {
    if (samePoint(bl_goal, goal)) {
      return true;
    }
  }
  return false;
}

bool ExploreNode::samePoint(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const
{
  const double tolerance = 0.1;
  return (std::fabs(a.x - b.x) < tolerance) &&
         (std::fabs(a.y - b.y) < tolerance) &&
         (std::fabs(a.z - b.z) < tolerance);
}

void ExploreNode::stop()
{
  // 修复：用!is_canceled()判断定时器是否激活
  if (!exploring_timer_->is_canceled()) {
    exploring_timer_->cancel();
  }
  RCLCPP_INFO(this->get_logger(), "Exploration stopped manually");
}
}  // namespace explore_lite

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<explore_lite::ExploreNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

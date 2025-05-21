#ifndef ROS2_TERM_PROJECT_CMD_PUBLISHER_H
#define ROS2_TERM_PROJECT_CMD_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "map.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <queue>
#include <cmath>
#include <tuple>
#include <vector>
#include <mutex>

class CmdPublisher : public rclcpp::Node {
public:
  // ✅ NodeOptions를 외부에서 받아 생성자에서 사용
  explicit CmdPublisher(const rclcpp::NodeOptions & options);

private:
  std::mutex path_mutex;
  void timer_tf_callback();
  void timer_cmd_callback();
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  double x, y;
  bool position_updated = false;
  Map map;

  rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  geometry_msgs::msg::Twist cmd_vel;
  double roll, pitch, yaw;

  float goal_x;
  float goal_y;
  bool goal_received;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  geometry_msgs::msg::Twist compute_dwa_command();

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub;
};

#endif // ROS2_TERM_PROJECT_CMD_PUBLISHER_H

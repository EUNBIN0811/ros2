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
  CmdPublisher();

private:
  std::mutex path_mutex;
  void timer_tf_callback();
  void timer_cmd_callback();
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);  // Occupancy Grid 구독

  // 현재 로봇 위치 정보 및 상태
  double x, y;
  bool position_updated = false;
  Map map;

  // Timer, Publisher, Subscriber 관련 객체들
  rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  geometry_msgs::msg::Twist cmd_vel;
  double roll, pitch, yaw;

  // RViz로부터 수신하는 2D Goal Pose 관련 변수들
  float goal_x;
  float goal_y;
  bool goal_received;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // DWA 알고리즘 함수 : 현재 상태와 맵 정보를 바탕으로 최적의 속도 커맨드 (선형, 각속도)를 계산
  geometry_msgs::msg::Twist compute_dwa_command();

  // 기존 A* 알고리즘 관련 함수들은 더 이상 사용하지 않으므로 삭제 혹은 주석 처리함
  // std::vector<std::pair<float, float>> generate_path_a_star(const std::pair<float, float>& start, const std::pair<float, float>& goal);
  // void follow_path(const std::vector<std::pair<float, float>>& path);
  // void visualize_path(const std::vector<std::pair<float, float>>& path);

  // (Optional) 경로/알고리즘 관련 시각화를 위한 publisher (필요시 활용)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub;
};

#endif // ROS2_TERM_PROJECT_CMD_PUBLISHER_H

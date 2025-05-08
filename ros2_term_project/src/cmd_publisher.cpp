#include "cmd_publisher.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher"), goal_x(0.0), goal_y(0.0), goal_received(false) {
  // Publisher for cmd_vel
  pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // (옵션) 경로 시각화용 Publisher – DWA에서는 꼭 필요하지 않다면 제거 가능
  path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

  // Map Subscriber (Occupancy Grid)
  sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "map", 10, std::bind(&CmdPublisher::map_callback, this, _1));

  // TF listener 설정
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // TF 업데이트 타이머
  timer_tf = this->create_wall_timer(50ms, std::bind(&CmdPublisher::timer_tf_callback, this));

  // 명령 실행 타이머 (DWA 알고리즘 실행)
  timer_cmd = this->create_wall_timer(100ms, std::bind(&CmdPublisher::timer_cmd_callback, this));

  // 2D Goal Pose Subscriber
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "move_base_simple/goal", 10,
      std::bind(&CmdPublisher::goal_callback, this, _1));
}

/////////////////////////////////////////// 콜백함수 ///////////////////////////////////////////

// TF를 사용해 현재 로봇 위치를 업데이트함
void CmdPublisher::timer_tf_callback() {
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link : %s", ex.what());
    return;
  }

  x = t.transform.translation.x;
  y = t.transform.translation.y;

  tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  position_updated = true;
}

// 2D Occupancy Grid 맵 업데이트 콜백
void CmdPublisher::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map.update(*msg);
}

// 2D Goal Pose 설정 콜백
void CmdPublisher::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_x = msg->pose.position.x;
  goal_y = msg->pose.position.y;
  goal_received = true;
  RCLCPP_INFO(this->get_logger(), "Received goal: x = %f, y = %f", goal_x, goal_y);
}

// DWA 알고리즘을 이용해 최적의 속도 커맨드를 계산하여 publish
void CmdPublisher::timer_cmd_callback() {
  if (!map.is_updated() || !goal_received || !position_updated) {
    RCLCPP_WARN(this->get_logger(), "Map, goal, or position not updated yet.");
    return;
  }

  // DWA를 통해 velocity command 계산
  geometry_msgs::msg::Twist cmd_vel = compute_dwa_command();
  pub_cmd->publish(cmd_vel);
}

// DWA 알고리즘을 통해 최적의 (선형, 각) 속도를 계산하는 함수
geometry_msgs::msg::Twist CmdPublisher::compute_dwa_command() {
  // 시뮬레이션 파라미터
  const float dt = 0.1f;          // 시뮬레이션 시간 간격
  const float sim_time = 2.0f;      // 시뮬레이션 전체 시간
  // 속도 제한 (터틀봇 버거에 맞게 조정 – 필요시 수정)
  const float max_linear_speed = 0.5f;   // m/s
  const float max_angular_speed = 1.0f;  // rad/s

  // 후보 속도 샘플링 간격
  const float linear_step = 0.05f;
  const float angular_step = 0.1f;

  // 비용 함수 가중치
  const float w_heading = 1.0f;    // 목표까지의 거리 비용
  const float w_clearance = 0.5f;  // 장애물와의 간격 비용 (높을수록 페널티)
  const float w_velocity = 0.1f;   // 선형 속도 보상

  // 충돌 판단 임계값
  const float collision_threshold = 0.2f; // m
  const float epsilon = 1e-5f;

  float best_score = std::numeric_limits<float>::infinity();
  float best_v = 0.0f;
  float best_w = 0.0f;

  // 선형 및 각속도의 모든 후보에 대해 시뮬레이션 수행
  for (float v = 0.0f; v <= max_linear_speed; v += linear_step) {
    for (float w = -max_angular_speed; w <= max_angular_speed; w += angular_step) {
      // 각 후보에 대해 로봇의 모션 시뮬레이션
      float sim_x = x;
      float sim_y = y;
      float sim_yaw = yaw;
      float min_clearance = std::numeric_limits<float>::infinity();

      for (float t = 0.0f; t < sim_time; t += dt) {
        sim_x += v * std::cos(sim_yaw) * dt;
        sim_y += v * std::sin(sim_yaw) * dt;
        sim_yaw += w * dt;

        float clearance;
        float obs_x, obs_y;
        map.get_distance_and_closest_obstacle(sim_x, sim_y, clearance, obs_x, obs_y);
        if (clearance < min_clearance) {
          min_clearance = clearance;
        }
        // 충돌 임계값 미만이면 해당 후보는 사용 불가
        if (clearance < collision_threshold) {
          min_clearance = 0.0f;
          break;
        }
      }

      // 충돌 위험이 있는 경우 스킵
      if (min_clearance <= collision_threshold) {
        continue;
      }

      // 시뮬레이션 후 최종 위치와 목표 사이의 유클리드 거리를 계산 (heading cost)
      float d_goal = std::hypot(goal_x - sim_x, goal_y - sim_y);
      float cost = w_heading * d_goal + w_clearance * (1.0f / (min_clearance + epsilon)) - w_velocity * v;

      if (cost < best_score) {
        best_score = cost;
        best_v = v;
        best_w = w;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "DWA chosen command: linear: %f, angular: %f", best_v, best_w);
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = best_v;
  cmd_vel.angular.z = best_w;
  return cmd_vel;
}

// 기존 A* 알고리즘 관련 함수는 더 이상 사용하지 않으므로 삭제 또는 주석 처리합니다.
/*
std::vector<std::pair<float, float>> CmdPublisher::generate_path_a_star(
    const std::pair<float, float>& start, const std::pair<float, float>& goal) {
  // A* 알고리즘 코드 (삭제됨)
}

void CmdPublisher::visualize_path(const std::vector<std::pair<float, float>>& path) {
  // 경로 시각화 코드 (삭제됨)
}

void CmdPublisher::follow_path(const std::vector<std::pair<float, float>>& path) {
  // 경로 추종 코드 (삭제됨)
}
*/

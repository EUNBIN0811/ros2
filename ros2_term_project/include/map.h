#ifndef ROS2_TERM_PROJECT_MAP_H
#define ROS2_TERM_PROJECT_MAP_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

class Map {
public:
  Map() = default;

  // 맵 업데이트 (Octomap → 2D Occupancy Grid)
  void update(const nav_msgs::msg::OccupancyGrid& map_msg);

  // 특정 위치에서 가장 가까운 장애물까지의 거리 및 좌표 계산
  void get_distance_and_closest_obstacle(float world_x, float world_y, 
                                         float& distance, float& obs_x, float& obs_y) const;

  [[nodiscard]] bool is_updated() const { return updated; }

private:
  bool updated = false;
  float map_resolution = 0.05; // 기본 맵 해상도 (m)
  int map_width = 0, map_height = 0;
  float map_origin_x = 0.0, map_origin_y = 0.0;
  cv::Mat label_map;
  cv::Mat map_data;       // Occupancy Grid 맵 (2D 행렬)
  cv::Mat distance_map;   // 거리 변환 맵 (2D 행렬)
};

#endif // ROS2_TERM_PROJECT_MAP_H

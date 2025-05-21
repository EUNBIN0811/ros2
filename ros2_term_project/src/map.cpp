#include "map.h"
#include <opencv2/imgproc.hpp>
#include <limits>

// Occupancy Grid 메시지를 이용하여 맵 데이터를 업데이트합니다.
void Map::update(const nav_msgs::msg::OccupancyGrid& map_msg) {
  // 맵 정보 업데이트
  map_width = map_msg.info.width;
  map_height = map_msg.info.height;
  map_resolution = map_msg.info.resolution;
  map_origin_x = map_msg.info.origin.position.x;
  map_origin_y = map_msg.info.origin.position.y;

  // OccupancyGrid의 data는 일반적으로 -1 (미확인), 0 (free), 100 (occupied) 등으로 구성됨.
  // 여기서는 occupancy==0를 free로 판단하여 255 (흰색)으로, 나머지는 장애물(0, 검은색)으로 설정합니다.
  cv::Mat temp(map_height, map_width, CV_8UC1);
  for (int i = 0; i < map_height; i++) {
    for (int j = 0; j < map_width; j++) {
      int idx = i * map_width + j;
      int8_t occ = map_msg.data[idx];
      if (occ == 0) {
        temp.at<uchar>(i, j) = 255;  // free space
      } else {
        temp.at<uchar>(i, j) = 0;    // obstacle (또는 미확인)
      }
    }
  }
  // 이진화한 맵 데이터를 내부 변수에 저장
  map_data = temp.clone();

  // OpenCV의 distanceTransform을 이용해 각 픽셀에서 가장 가까운 장애물(픽셀값 0)까지의 거리를 계산합니다.
  // 또한, 레이블맵(label_map)을 구해 각 픽셀에 대해 가장 가까운 장애물 픽셀의 인덱스를 반환합니다.
  cv::Mat dist, labels;
  cv::distanceTransform(temp, dist, labels, cv::DIST_L2, 3);

  // 거리 값은 픽셀 단위이므로 실제 거리(미터 단위)를 위해 해상도를 곱합니다.
  dist = dist * map_resolution;

  distance_map = dist;
  label_map = labels; // 가장 가까운 장애물의 픽셀 인덱스를 저장

  updated = true;
}

// 주어진 월드 좌표 (world_x, world_y)에서 가장 가까운 장애물까지의 거리와 해당 장애물의 월드 좌표(obs_x, obs_y)를 계산합니다.
void Map::get_distance_and_closest_obstacle(float world_x, float world_y, 
                                              float& distance, float& obs_x, float& obs_y) const {
  // 월드 좌표를 맵 인덱스로 변환 (맵의 원점과 해상도 이용)
  int map_j = static_cast<int>((world_x - map_origin_x) / map_resolution);
  int map_i = static_cast<int>((world_y - map_origin_y) / map_resolution);

  // 인덱스가 맵 범위를 벗어나면 큰 값(inf)으로 처리
  if (map_i < 0 || map_i >= map_height || map_j < 0 || map_j >= map_width) {
    distance = std::numeric_limits<float>::infinity();
    obs_x = world_x;
    obs_y = world_y;
    return;
  }

  // 해당 셀의 장애물까지의 거리를 가져옴
  distance = distance_map.at<float>(map_i, map_j);

  // 레이블맵을 이용하여 가장 가까운 장애물 픽셀의 선형 인덱스를 가져옴
  int label = label_map.at<int>(map_i, map_j);

  // 선형 인덱스를 2D 인덱스로 변환 (행: obs_i, 열: obs_j)
  int obs_i = label / map_width;
  int obs_j = label % map_width;

  // 맵 인덱스를 월드 좌표로 변환 (각 셀의 중심 좌표)
  obs_x = map_origin_x + (obs_j + 0.5f) * map_resolution;
  obs_y = map_origin_y + (obs_i + 0.5f) * map_resolution;
}

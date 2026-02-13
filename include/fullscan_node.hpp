#ifndef FULLSCAN_NODE_HPP
#define FULLSCAN_NODE_HPP

// C++ Standard Libraries
#include <string>
#include <vector>
#include <cmath>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

// PointCloud2의 data 레이아웃과 1:1 대응하는 구조체
struct Point
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};
static_assert(sizeof(Point) == 12, "Point struct must be 12 bytes");

class FullScanNode : public rclcpp::Node
{
public:
  FullScanNode();

private:
  // --- 콜백 ---
  void on_partial(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // --- 핵심 로직 함수 ---
  double compute_median_azimuth(const Point * points, size_t count);
  bool is_new_zone(double azimuth);
  void emit_full_scan(const builtin_interfaces::msg::Time & stamp);
  void reset_buffer();

  // --- ROS2 통신 ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // --- 누적 버퍼 ---
  std::vector<Point> buffer_;
  size_t write_idx_ = 0;

  // --- zone 감지용 상태 ---
  std::vector<double> seen_zones_;
  double prev_azimuth_ = -999.0;
  double first_partial_stamp_ = 0.0;

  // --- 파라미터 ---
  std::string frame_id_ = "velodyne";
  int num_zones_ = 3;
  double zone_tolerance_deg_ = 20.0;
  double timeout_sec_ = 0.5;
  int max_points_ = 300000;
};

#endif

#include "fullscan_node.hpp"
#include <algorithm>

// =============================================================
// 생성자
// =============================================================
FullScanNode::FullScanNode()
: Node("fullscan_node")
{
  // 파라미터 선언
  this->declare_parameter("input_topic", "/point_cloud");
  this->declare_parameter("output_topic", "/velodyne_points");
  this->declare_parameter("frame_id", "velodyne");
  this->declare_parameter("num_zones", 3);
  this->declare_parameter("zone_tolerance_deg", 20.0);
  this->declare_parameter("timeout_sec", 0.5);
  this->declare_parameter("max_points", 300000);

  // 파라미터 읽기
  auto input_topic = this->get_parameter("input_topic").as_string();
  auto output_topic = this->get_parameter("output_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  num_zones_ = this->get_parameter("num_zones").as_int();
  zone_tolerance_deg_ = this->get_parameter("zone_tolerance_deg").as_double();
  timeout_sec_ = this->get_parameter("timeout_sec").as_double();
  max_points_ = this->get_parameter("max_points").as_int();

  // 버퍼 사전 할당
  buffer_.reserve(max_points_);
  seen_zones_.reserve(num_zones_);

  // Subscriber (BEST_EFFORT — Isaac Sim 매칭)
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&FullScanNode::on_partial, this, std::placeholders::_1));

  // Publisher (RELIABLE — Nav2 호환)
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic,
    rclcpp::QoS(10).reliable());

  RCLCPP_INFO(this->get_logger(),
    "FullScan ready: %s -> %s (frame=%s, zones=%d)",
    input_topic.c_str(), output_topic.c_str(), frame_id_.c_str(), num_zones_);
}

// =============================================================
// 콜백: 부분 스캔 수신
// =============================================================
void FullScanNode::on_partial(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 1. PointCloud2 → Point* 변환 (zero-copy)
  const auto * points = reinterpret_cast<const Point *>(msg->data.data());
  size_t count = msg->width * msg->height;

  if (count == 0) {
    return;
  }

  // 2. median azimuth 계산
  double current_azimuth = compute_median_azimuth(points, count);

  // 3. 중복 프레임 필터링 (이전 partial과 같으면 스킵)
  if (std::abs(current_azimuth - prev_azimuth_) < 5.0) {
    return;
  }
  prev_azimuth_ = current_azimuth;

  // 4. 타임스탬프 (초 단위)
  double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // 5. zone 등록
  bool new_zone = is_new_zone(current_azimuth);

  // 디버그 로그
  RCLCPP_INFO(this->get_logger(),
    "zones=%zu/%d, az=%.1f, new=%s, pts=%zu",
    seen_zones_.size(), num_zones_, current_azimuth,
    new_zone ? "YES" : "dup", count);

  // 6. 현재 partial을 버퍼에 먼저 추가
  buffer_.insert(buffer_.end(), points, points + count);
  write_idx_ += count;

  // 7. 타임스탬프 갱신
  if (first_partial_stamp_ <= 0.0) {
    first_partial_stamp_ = stamp_sec;
  }

  // 8. 모든 zone이 모였으면 emit 후 return
  if (static_cast<int>(seen_zones_.size()) >= num_zones_) {
    RCLCPP_INFO(this->get_logger(),
      ">>> All %d zones collected! Publishing full scan", num_zones_);
    emit_full_scan(msg->header.stamp);
    reset_buffer();
    return;
  }
  // timeout 안전장치
  if (first_partial_stamp_ > 0.0 &&
      (stamp_sec - first_partial_stamp_) > timeout_sec_) {
    RCLCPP_WARN(this->get_logger(),
      ">>> TIMEOUT: only %zu/%d zones, publishing partial",
      seen_zones_.size(), num_zones_);
    emit_full_scan(msg->header.stamp);
    reset_buffer();
  }
}

// =============================================================
// azimuth 중간값 계산
// =============================================================
double FullScanNode::compute_median_azimuth(const Point * points, size_t count)
{
  std::vector<double> azimuths(count);

  for (size_t i = 0; i < count; i++) {
    azimuths[i] = std::atan2(points[i].y, points[i].x) * 180.0 / M_PI;
  }

  // std::nth_element로 O(n) median
  size_t mid = count / 2;
  std::nth_element(azimuths.begin(), azimuths.begin() + mid, azimuths.end());

  return azimuths[mid];
}

// =============================================================
// 새로운 zone인지 판별 (기존 zone과 ±tolerance 이내면 같은 zone)
// =============================================================
bool FullScanNode::is_new_zone(double azimuth)
{
  for (const auto & zone : seen_zones_) {
    if (std::abs(azimuth - zone) < zone_tolerance_deg_) {
      return false;
    }
  }
  seen_zones_.push_back(azimuth);
  return true;
}

// =============================================================
// full scan publish
// =============================================================
void FullScanNode::emit_full_scan(const builtin_interfaces::msg::Time & stamp)
{
  if (write_idx_ == 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 out_msg;

  // header
  out_msg.header.stamp = stamp;
  out_msg.header.frame_id = frame_id_;

  // 크기 정보
  out_msg.height = 1;
  out_msg.width = write_idx_;
  out_msg.point_step = sizeof(Point);
  out_msg.row_step = out_msg.point_step * out_msg.width;
  out_msg.is_dense = true;

  // fields 정의 (x, y, z)
  sensor_msgs::msg::PointField field;

  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  out_msg.fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  out_msg.fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  out_msg.fields.push_back(field);

  // data 복사
  const auto * raw = reinterpret_cast<const uint8_t *>(buffer_.data());
  out_msg.data.assign(raw, raw + out_msg.row_step);

  pub_->publish(out_msg);

  RCLCPP_INFO(this->get_logger(),
    "Full scan published: %zu points", write_idx_);
}

// =============================================================
// 버퍼 초기화
// =============================================================
void FullScanNode::reset_buffer()
{
  buffer_.clear();
  write_idx_ = 0;
  seen_zones_.clear();
  prev_azimuth_ = -999.0;
  first_partial_stamp_ = 0.0;
}

// =============================================================
// main
// =============================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FullScanNode>());
  rclcpp::shutdown();
  return 0;
}

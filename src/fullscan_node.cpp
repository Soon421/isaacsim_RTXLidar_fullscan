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
  this->declare_parameter("target_hz", 10.0);
  this->declare_parameter("ghost_range_min", 0.4);
  this->declare_parameter("ghost_elev_bins_max", 40);
  this->declare_parameter("ghost_z_range_max", 20.0);
  this->declare_parameter("debug_mode", false);

  // 파라미터 읽기
  auto input_topic = this->get_parameter("input_topic").as_string();
  auto output_topic = this->get_parameter("output_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  num_zones_ = this->get_parameter("num_zones").as_int();
  zone_tolerance_deg_ = this->get_parameter("zone_tolerance_deg").as_double();
  timeout_sec_ = this->get_parameter("timeout_sec").as_double();
  max_points_ = this->get_parameter("max_points").as_int();
  target_hz_ = this->get_parameter("target_hz").as_double();
  ghost_range_min_ = this->get_parameter("ghost_range_min").as_double();
  ghost_elev_bins_max_ = this->get_parameter("ghost_elev_bins_max").as_int();
  ghost_z_range_max_ = this->get_parameter("ghost_z_range_max").as_double();
  debug_mode_ = this->get_parameter("debug_mode").as_bool();
  min_publish_interval_ = 1.0 / target_hz_;

  // 버퍼 사전 할당
  buffer_.reserve(max_points_);
  seen_zones_.reserve(num_zones_);

  // Subscriber (RELIABLE — point_cloud_gt 호환)
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic,
    rclcpp::QoS(10).reliable(),
    std::bind(&FullScanNode::on_partial, this, std::placeholders::_1));

  // Publisher (RELIABLE — Nav2 호환)
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic,
    rclcpp::QoS(10).reliable());

  RCLCPP_INFO(this->get_logger(),
    "FullScan ready: %s -> %s (frame=%s, zones=%d, target_hz=%.1f)",
    input_topic.c_str(), output_topic.c_str(), frame_id_.c_str(), num_zones_, target_hz_);
}

// =============================================================
// 콜백: 부분 스캔 수신
// =============================================================
void FullScanNode::on_partial(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 발행 대기 중이면 새 partial 무시
  if (pending_publish_) {
    return;
  }

  // 1. PointCloud2 → Point* 변환 (zero-copy)
  const auto * points = reinterpret_cast<const Point *>(msg->data.data());
  size_t count = msg->width * msg->height;

  if (count == 0) {
    return;
  }

  // 2. median azimuth 계산
  double current_azimuth = compute_median_azimuth(points, count);

  // 3. 디버그 출력 (debug_mode일 때)
  if (debug_mode_) {
    debug_partial(points, count, current_azimuth);
  }

  // 4. ghost 필터 적용 (항상)
  if (is_ghost_partial(points, count)) {
    if (!debug_mode_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ghost rejected (%zu pts)", count);
    }
    return;
  }

  // 4. 중복 프레임 필터링 (이전 partial과 같으면 스킵)
  if (std::abs(current_azimuth - prev_azimuth_) < 5.0) {
    return;
  }
  prev_azimuth_ = current_azimuth;

  // 5. 타임스탬프 (초 단위)
  double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // 6. zone 등록
  is_new_zone(current_azimuth);

  // 7. 현재 partial을 버퍼에 먼저 추가
  buffer_.insert(buffer_.end(), points, points + count);
  write_idx_ += count;

  // 8. 타임스탬프 갱신
  if (first_partial_stamp_ <= 0.0) {
    first_partial_stamp_ = stamp_sec;
  }

  // 9. 모든 zone이 모였으면 emit 후 return
  if (static_cast<int>(seen_zones_.size()) >= num_zones_) {
    try_publish(msg->header.stamp);
    return;
  }
  // timeout 안전장치
  if (first_partial_stamp_ > 0.0 &&
      (stamp_sec - first_partial_stamp_) > timeout_sec_) {
    try_publish(msg->header.stamp);
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
// ghost 판별: (range_min < ghost_range_min_) OR (elev_bins > ghost_elev_bins_max_)
// =============================================================
bool FullScanNode::is_ghost_partial(const Point * points, size_t count)
{
  // 1. 최근접점 확인 + z 범위 계산 (single pass)
  float threshold_sq = static_cast<float>(ghost_range_min_ * ghost_range_min_);
  float z_min = 1e9f;
  float z_max = -1e9f;

  for (size_t i = 0; i < count; i++) {
    float range_sq = points[i].x * points[i].x +
                     points[i].y * points[i].y +
                     points[i].z * points[i].z;
    if (range_sq < threshold_sq) {
      return true;
    }
    if (points[i].z < z_min) z_min = points[i].z;
    if (points[i].z > z_max) z_max = points[i].z;
  }

  // 2. z 범위 확인
  if (static_cast<double>(z_max - z_min) > ghost_z_range_max_) {
    return true;
  }

  // 3. elevation bins 계산 (0.1° 해상도, -30° ~ +30°)
  constexpr int NUM_ELEV_BINS = 600;
  bool elev_occupied[NUM_ELEV_BINS] = {};

  for (size_t i = 0; i < count; i++) {
    double rh = std::sqrt(static_cast<double>(points[i].x) * points[i].x +
                          static_cast<double>(points[i].y) * points[i].y);
    double elevation = std::atan2(static_cast<double>(points[i].z), rh) * 180.0 / M_PI;
    int bin = static_cast<int>((elevation + 30.0) / 0.1);
    if (bin >= 0 && bin < NUM_ELEV_BINS) {
      elev_occupied[bin] = true;
    }
  }

  int unique_bins = 0;
  for (int i = 0; i < NUM_ELEV_BINS; i++) {
    if (elev_occupied[i]) unique_bins++;
  }

  return unique_bins > ghost_elev_bins_max_;
}

// =============================================================
// 디버그: partial 포인트 특성 상세 분석
// =============================================================
void FullScanNode::debug_partial(const Point * points, size_t count, double median_az)
{
  double range_min = 1e9;
  double range_max = 0.0;
  double range_sum = 0.0;
  int near_zero_count = 0;
  int short_range_count = 0;
  float z_min = 1e9f;
  float z_max = -1e9f;

  constexpr int NUM_ELEV_BINS = 600;
  bool elev_occupied[NUM_ELEV_BINS] = {};

  for (size_t i = 0; i < count; i++) {
    float x = points[i].x;
    float y = points[i].y;
    float z = points[i].z;

    double range_3d = std::sqrt(x * x + y * y + z * z);
    if (range_3d < range_min) range_min = range_3d;
    if (range_3d > range_max) range_max = range_3d;
    range_sum += range_3d;
    if (range_3d < 0.1) near_zero_count++;
    if (range_3d < 1.0) short_range_count++;

    if (z < z_min) z_min = z;
    if (z > z_max) z_max = z;

    double range_h = std::sqrt(x * x + y * y);
    double elevation = std::atan2(z, range_h) * 180.0 / M_PI;
    int bin = static_cast<int>((elevation + 30.0) / 0.1);
    if (bin >= 0 && bin < NUM_ELEV_BINS) {
      elev_occupied[bin] = true;
    }
  }

  int elev_bins = 0;
  for (int i = 0; i < NUM_ELEV_BINS; i++) {
    if (elev_occupied[i]) elev_bins++;
  }

  double range_mean = range_sum / count;
  double density = static_cast<double>(count) / std::max(elev_bins, 1);
  double z_range = static_cast<double>(z_max - z_min);

  bool is_ghost = (range_min < ghost_range_min_) ||
                  (elev_bins > ghost_elev_bins_max_) ||
                  (z_range > ghost_z_range_max_);

  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] pts=%zu az=%.1f | range=[%.2f, %.2f, %.2f] | elev_bins=%d density=%.1f | "
    "z=[%.2f, %.2f] zR=%.1f | near_zero=%d short_range=%d | %s",
    count, median_az, range_min, range_mean, range_max, elev_bins, density,
    z_min, z_max, z_range, near_zero_count, short_range_count,
    is_ghost ? ">>> GHOST <<<" : "OK");
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
// rate limiting: 최소 간격 보장 후 publish
// =============================================================
void FullScanNode::try_publish(const builtin_interfaces::msg::Time & stamp)
{
  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(now - last_emit_time_).count();

  if (elapsed >= min_publish_interval_) {
    // 충분한 시간이 지남 → 즉시 발행
    emit_full_scan(stamp);
    reset_buffer();
    last_emit_time_ = now;
  } else {
    // 너무 빠름 → 남은 시간만큼 대기 후 발행
    double delay = min_publish_interval_ - elapsed;
    pending_stamp_ = stamp;
    pending_publish_ = true;

    rate_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(delay)),
      [this]() {
        rate_timer_->cancel();
        emit_full_scan(pending_stamp_);
        reset_buffer();
        last_emit_time_ = std::chrono::steady_clock::now();
        pending_publish_ = false;
      });
  }
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

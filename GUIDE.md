# Isaac Sim Partial Scan → Full Scan 구현 가이드

## 목표

Isaac Sim `fullScan=False`(부분 스캔, 고속) → 별도 ROS2 C++ 노드에서 누적 → 360도 PointCloud2 publish

```
[Isaac Sim] --partial PointCloud2--> /point_cloud
                    |
          [fullscan 노드] -- 누적 --
                    |
              /velodyne_points --> [Nav2]
```

## 패키지 구조

```
fullscan/
├── package.xml
├── CMakeLists.txt
├── include/fullscan/
│   └── fullscan_node.hpp
├── src/
│   └── fullscan_node.cpp
└── launch/
    └── fullscan.launch.py
```

---

## Step 1: 패키지 스캐폴딩

### package.xml
- 필요한 depend: `rclcpp`, `sensor_msgs`, `std_msgs`
- build_type: `ament_cmake`

### CMakeLists.txt
- `find_package`: rclcpp, sensor_msgs, std_msgs
- `add_executable`: fullscan_node src/fullscan_node.cpp
- `target_include_directories`: include 디렉토리
- `ament_target_dependencies`: rclcpp sensor_msgs std_msgs
- `install`: 실행파일 + launch 디렉토리

### 힌트
- `ros2 pkg create --build-type ament_cmake fullscan --dependencies rclcpp sensor_msgs std_msgs`로 시작하면 편함
- 단, 위 명령은 include/ 구조를 자동으로 안 만들어줄 수 있으니 직접 `include/fullscan/` 디렉토리 생성

---

## Step 2: fullscan_node.hpp

여기서 선언해야 할 것들:

### 1) Point 구조체
- `float x, y, z, intensity` — 총 16바이트
- PointCloud2의 data 레이아웃과 1:1 대응되게
- 힌트: `#pragma pack(push, 1)` 또는 `__attribute__((packed))` 고려 (패딩 방지)

### 2) 노드 클래스 (rclcpp::Node 상속)
- **멤버 변수로 필요한 것들:**
  - subscriber (PointCloud2)
  - publisher (PointCloud2)
  - 누적 버퍼: `std::vector<Point>` — 생성자에서 `reserve(max_points)`
  - write 위치 추적용 변수
  - 이전 partial의 median azimuth (wrap 감지용)
  - partial 카운트, 첫 partial 타임스탬프 (fallback용)
  - ROS2 파라미터들 (input_topic, output_topic, frame_id, wrap_threshold 등)

- **멤버 함수로 필요한 것들:**
  - `void on_partial(const sensor_msgs::msg::PointCloud2::SharedPtr msg)` — 콜백
  - `double compute_median_azimuth(const Point* points, size_t count)` — azimuth 중간값
  - `bool detect_wrap(double current_azimuth)` — wrap-around 판정
  - `void emit_full_scan(builtin_interfaces::msg::Time stamp)` — 버퍼 → PointCloud2 publish
  - `void reset_buffer()` — 버퍼 초기화 (write 위치만 0으로)

---

## Step 3: fullscan_node.cpp

### 콜백 흐름 (on_partial)
```
1. msg->data에서 Point* 로 reinterpret_cast (zero-copy 수신)
2. 포인트 수 계산: msg->width * msg->height
3. median azimuth 계산: atan2(y, x) → 정렬 → 중간값
4. wrap-around 체크:
   - 이전 median - 현재 median > threshold(90도) → 회전 완료!
   - 또는 partial 카운트 >= max → 강제 완료
   - 또는 시간 초과 → 강제 완료
5. 회전 완료 시:
   - emit_full_scan() — 현재 버퍼를 publish
   - reset_buffer()
   - ★ 현재 partial은 다음 회전의 첫 데이터이므로 reset 후 추가
6. 버퍼에 현재 partial append (memcpy 또는 insert)
7. partial 카운트++, 이전 azimuth 갱신
```

### emit_full_scan 구현 힌트
- `sensor_msgs::msg::PointCloud2` 메시지 새로 생성
- `header.frame_id` = 파라미터에서 가져온 frame_id
- `header.stamp` = 마지막 partial의 타임스탬프
- `height = 1`, `width = 포인트 수`
- `point_step = 16` (sizeof(Point))
- `row_step = point_step * width`
- `is_dense = true`
- `fields`: x(offset=0), y(offset=4), z(offset=8), intensity(offset=12) — 전부 FLOAT32
- `data`: 버퍼를 `reinterpret_cast<uint8_t*>`로 캐스팅해서 복사

### median azimuth 구현 힌트
- 모든 점의 `atan2(y, x)` 계산 → radian → degree
- `std::nth_element`로 O(n) median 구하기 (전체 정렬보다 빠름)

### QoS 주의사항
- Subscriber: `rclcpp::SensorDataQoS()` → BEST_EFFORT (Isaac Sim 매칭)
- Publisher: `rclcpp::QoS(10).reliable()` → Nav2 호환
- **QoS 안 맞으면 토픽이 보여도 메시지가 안 옴!** (흔한 함정)

### main 함수
- `rclcpp::init` → 노드 생성 → `rclcpp::spin` → `rclcpp::shutdown`

---

## Step 4: fullscan.launch.py

- `Node` action으로 fullscan_node 실행
- `parameters`에서 input_topic, output_topic, frame_id 등 직접 지정
- `use_sim_time: True` 필수

---

## 파라미터 정리

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `input_topic` | `/point_cloud` | Isaac Sim 부분 스캔 토픽 |
| `output_topic` | `/velodyne_points` | Nav2용 full scan 토픽 |
| `frame_id` | `velodyne` | TF 프레임 |
| `wrap_threshold_deg` | `90.0` | wrap-around 판정 임계값 |
| `max_partials` | `12` | 안전 상한 |
| `timeout_sec` | `0.2` | 강제 emit 타임아웃 |
| `max_points` | `300000` | 버퍼 reserve 크기 |

---

## 주의사항 / 흔한 실수

1. **wrap-around 시 현재 partial 처리**: emit 후 reset하고, 현재 partial을 새 버퍼에 넣어야 함. 안 하면 매 회전 시작 부분이 빠짐
2. **QoS 불일치**: Isaac Sim은 BEST_EFFORT, subscriber도 BEST_EFFORT여야 수신 가능
3. **use_sim_time**: 안 켜면 타임스탬프 비교가 꼬임
4. **첫 번째 스캔**: 불완전할 수 있음 (Isaac Sim 특성). 첫 emit은 360도가 아닐 수 있는데 정상
5. **Point 구조체 패딩**: float 4개면 보통 패딩 없지만, 확실히 하려면 `static_assert(sizeof(Point) == 16)`

---

## 검증 순서

1. `colcon build --packages-select fullscan` — 빌드 확인
2. `ros2 topic echo /point_cloud --field header.stamp` — Isaac Sim partial 수신 확인
3. `ros2 run fullscan fullscan_node` — 노드 실행
4. `ros2 topic hz /velodyne_points` — LiDAR 회전율(~10Hz)과 일치하는지 확인
5. **RViz2**: `/velodyne_points` 시각화 → 360도 원형 확인 (wedge 아님)
6. **Nav2 costmap**: 장애물 반영 확인

---

## 레퍼런스

- Velodyne ROS driver (azimuth wrap 감지 패턴): https://github.com/ros-drivers/velodyne
- ros-perception/laser_assembler (버퍼 관리): https://github.com/ros-perception/laser_assembler
- Isaac Sim RTX LiDAR 문서: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx_lidar.html

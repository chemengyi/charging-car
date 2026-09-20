// ============================================================================
// costmap_generator_node.cpp 修改补丁
//
// 共 3 处修改:
//   1. 构造函数: 添加 /task_planner/dynamic_parking_lot 订阅
//   2. 新增回调: onDynamicParkingLot (追加到 primitives_points_)
//   3. isActive(): 有动态 parking_lot 时直接返回 true
//
// 在 costmap_generator.hpp (或 .h) 的 private 区域添加:
//   rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_dynamic_parking_lot_;
//   bool has_dynamic_parking_lot_ = false;
//   void onDynamicParkingLot(const geometry_msgs::msg::Polygon::ConstSharedPtr msg);
//
// 在 .hpp 的 #include 区域确保有:
//   #include <geometry_msgs/msg/polygon.hpp>
// ============================================================================


// ──────────────────────────────────────────
// 修改1: 构造函数中, 在其他 subscription 之后添加:
// ──────────────────────────────────────────
/*
  sub_dynamic_parking_lot_ = create_subscription<geometry_msgs::msg::Polygon>(
    "/task_planner/dynamic_parking_lot",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&CostmapGenerator::onDynamicParkingLot, this, std::placeholders::_1));
  has_dynamic_parking_lot_ = false;
*/


// ──────────────────────────────────────────
// 修改2: 新增回调函数 (放在其他回调函数附近)
// ──────────────────────────────────────────
/*
void CostmapGenerator::onDynamicParkingLot(const geometry_msgs::msg::Polygon::ConstSharedPtr msg)
{
  if (!msg || msg->points.size() < 3) {
    RCLCPP_WARN(get_logger(), "[动态ParkingLot] 无效多边形 (点数=%zu)",
      msg ? msg->points.size() : 0);
    return;
  }

  // 追加到 primitives_points_ (不清空地图的 parking_lot!)
  std::vector<geometry_msgs::msg::Point> pts;
  for (const auto & p32 : msg->points) {
    geometry_msgs::msg::Point p;
    p.x = p32.x;
    p.y = p32.y;
    p.z = p32.z;
    pts.push_back(p);
  }
  primitives_points_.push_back(pts);

  // 追加到 parking_lots 用于发布
  parking_lots.polygon_array.push_back(*msg);
  pub_parking_lots->publish(parking_lots);

  has_dynamic_parking_lot_ = true;

  RCLCPP_INFO(get_logger(),
    "[动态ParkingLot] 已追加停车区域 (%zu角点), 总primitives=%zu",
    msg->points.size(), primitives_points_.size());
}
*/


// ──────────────────────────────────────────
// 修改3: isActive() 函数, 在开头添加动态判断
// ──────────────────────────────────────────
/*
bool CostmapGenerator::isActive()
{
  if (!lanelet_map_) {
    return false;
  }

  // ★ 新增: 如果有动态 parking_lot, 直接激活
  if (has_dynamic_parking_lot_) {
    return true;
  }

  // ↓↓↓ 以下是原有逻辑, 不修改 ↓↓↓
  if (activate_by_scenario_) {
    if (scenario_) {
      const auto & s = scenario_->activating_scenarios;
      if (
        std::find(std::begin(s), std::end(s), tier4_planning_msgs::msg::Scenario::PARKING) !=
        std::end(s)) {
        return true;
      }
    }
    return false;
  } else {
    const auto & current_pose_wrt_map = getCurrentPose(tf_buffer_, this->get_logger());
    if (!current_pose_wrt_map) return false;
    return isInParkingLot(lanelet_map_, current_pose_wrt_map->pose);
  }
}
*/

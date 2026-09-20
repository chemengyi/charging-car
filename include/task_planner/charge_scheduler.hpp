#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <random>
#include <limits>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_core/geometry/LineString.h>

namespace state_machine
{

using geometry_msgs::msg::Point;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_mapping_msgs::msg::HADMapBin;

// 目标车
struct TargetVehicle
{
  Point pos;
  double remaining_battery;  // 剩余电量 (0-100 %)
  double capacity_kwh;       // ★ 电池容量(kWh, 决定充电需求)
  std::string vehicle_type;  // ★ 车辆类型(如 "48V_100Ah")
  double deadline;           // τ_i = α·b_i (min, 趟内相对)
  int original_index;
};

using Trip = std::vector<int>;      // 车索引序列(不含驿站)
using Plan = std::vector<Trip>;     // 多趟方案

// 方案评估结果
struct PlanResult
{
  bool feasible = false;         // 满足所有硬约束(C2/C4/C6)
  int num_trips = 0;             // 趟数 m
  double total_distance = 0.0;   // 总路程 (m)
  double total_time = 0.0;       // 总时间 (min)
  int on_time_count = 0;         // 按时车辆数(C8降级用)
  int served_count = 0;
  double total_overtime = 0.0;   // ★ 总超时量(min, 所有超时车的超时之和, 降级最小化)
  std::vector<double> arrival_time;   // 每车到达时刻(全局累计), -1未服务
  std::vector<bool> on_time;
  std::vector<bool> charge_drive_battery; // z^k
};

class ChargeScheduler : public rclcpp::Node
{
public:
  explicit ChargeScheduler(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ChargeScheduler() = default;

private:
  // 回调
  void onMap(const HADMapBin::ConstSharedPtr msg);
  void onTrigger(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void onVehicleServed(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void onArrivedStation(const std_msgs::msg::Bool::ConstSharedPtr msg);  // 充电车回站反馈
  void sendReturnToStation();   // 发返程目标(趟间回站换电池)
  void startNextTrip();         // 开始下一趟
  void onTimer();

  // 地图/中心线
  void extractCenterlines();
  int findNearestLine(const Point & p);
  int findNearestPointIdx(int line_idx, const Point & p);
  double euclid(const Point & a, const Point & b);
  bool isSameDirection(int la, int lb);
  int findOppositeLane(int cur);
  double distAlongLine(int line_idx, int ia, int ib);

  // 档位3 精确路径距离
  double computePathDistance(const Point & from, const Point & to);

  // 能量感知评估器: 按访问顺序动态分趟模拟
  PlanResult simulateOrder(const std::vector<int> & order, Plan & out_trips);
  double chargeEnergyNeed(int vehicle_idx);   // 目标车充到80%需消耗充电车的电(kWh)
  double chargeTimeMin(int vehicle_idx);      // 充这辆车的时间(min)
  double tripDistance(const Trip & trip);

  // 精确求解 (n≤8): 枚举访问顺序全排列
  void solveExact();
  // 启发式 (n>8): 顺序LNS
  void solveHeuristic();

  // 方案比较(字典序)
  bool isBetter(const PlanResult & a, const PlanResult & b);

  // 主调度
  void runSchedule();
  void publishResult(const Plan & plan, const PlanResult & res);

  // 逐个发送
  void dispatchNext();
  void publishAllMarkers();

  // 订阅/发布
  rclcpp::Subscription<HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr served_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_station_sub_;  // 回站反馈
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_sequence_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_return_goal_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_task_complete_;  // 全部结束信号
  rclcpp::TimerBase::SharedPtr timer_;

  // 地图
  lanelet::LaneletMapPtr lanelet_map_ = nullptr;
  lanelet::ConstLanelets road_lanelets_;
  std::vector<std::vector<Point>> all_centerlines_;
  bool map_loaded_ = false;
  bool scheduled_ = false;

  // 配置参数
  Point charging_station_;
  double avg_speed_;                  // m/min
  double alpha_;                      // τ_i = α·b_i

  // ★★★ 能量感知调度模型 (一整块电池, kWh) ★★★
  double battery_capacity_kwh_;       // 充电车总电池容量 31.3 kWh
  double charge_power_kw_;            // 给目标车充电功率 100 kW
  double charge_efficiency_;         // 充电效率 0.97
  double drive_consume_kwh_per_km_;  // 自身行驶 0.6 kWh/km
  double station_charge_power_kw_;   // 充电桩功率 120 kW
  double soc_low_threshold_;         // 20% 阈值
  double soc_depart_threshold_;      // 80% 后续趟出发/充到
  double target_charge_target_;      // 目标车充到 80%
  // 换车加权评分权重
  double w_distance_;                // 距离权重(最高)
  double w_battery_;                 // 电量低权重
  double w_demand_;                  // 需求小权重

  std::vector<TargetVehicle> target_vehicles_;

  // 结果 + 逐个发送
  Plan final_plan_;
  PlanResult final_result_;
  std::vector<int> planned_order_;
  int dispatch_index_ = 0;
  bool dispatching_ = false;
  // ★ 分趟发送状态
  int current_trip_ = 0;        // 当前第几趟(0-based)
  int trip_vehicle_idx_ = 0;    // 当前趟内第几辆(0-based)
  bool waiting_return_ = false;  // 正在等充电车回站(趟间换电池)
};

} // namespace state_machine

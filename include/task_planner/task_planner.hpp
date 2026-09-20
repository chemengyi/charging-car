#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <limits>
#include <vector>
#include <queue>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/bool.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <promote_planning_msgs/msg/state_machine.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_core/geometry/LineString.h>

// ★ routing graph (虚线捷径决策 - 全局路径长度计算)
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include "utils/utils.hpp"

namespace state_machine
{

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using promote_planning_msgs::msg::StateMachine;

struct RoadReference
{
  Point nearest_point;
  double ref_yaw = 0.0;
  double ref_dx = 0.0;
  double ref_dy = 0.0;
  int line_idx = -1;
  int point_idx = -1;
  bool valid = false;
};

enum class TaskType { EM, ASTAR };

struct PlanTask
{
  TaskType type;
  PoseStamped goal;
  std::string description;
};

enum class PlannerState
{
  IDLE,
  OUTBOUND_EM,
  OUTBOUND_ASTAR,
  CHARGING,
  RETURN_ASTAR,
  RETURN_EM,
  COMPLETED
};

class task_planner : public rclcpp::Node
{
public:
  explicit task_planner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~task_planner() = default;

private:
  // 回调
  void onTimer();
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onGoal(const PoseStamped::ConstSharedPtr msg);
  void onAstarCompleted(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void onTargetVehicle(const PredictedObjects::ConstSharedPtr msg);
  void onMatchedVehicle(const PredictedObjects::ConstSharedPtr msg);
  void onMap(const HADMapBin::ConstSharedPtr msg);
  void onChargeComplete(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void onReturnGoal(const PoseStamped::ConstSharedPtr msg);

  // 工具
  void publishState(uint8_t val, const geometry_msgs::msg::Pose & p);
  bool isDataReady();
  void extractRoadCenterlines();
  void mergeCenterlines();

  // 虚线检测
  bool isDashedLeftBoundary(int line_idx, int point_idx);
  Point projectPointOnCenterline(int line_idx, const Point & pt);
  bool isMatchPointAhead(int line_idx, const Point & veh, int match_idx);
  void publishMarkers();
  PoseStamped makePose(double x, double y, double yaw);
  void sendEmGoal(const PoseStamped & g);
  void clearEmGoal();
  void clearAllGoals();
  void sendAstarGoal(const PoseStamped & g);
  double distanceTo(const PoseStamped & g);

  // 道路解析
  int findNearestCenterlineIndex(const Point & pt);
  Point findNearestPointOnLine(int idx, const Point & pt);
  RoadReference computeRoadRef(int idx, const Point & pt);
  int findPointIndexOnLine(int line_idx, const Point & pt);
  bool isSameDirection(int la, int lb);
  int findOppositeLane(int cur);
  bool isBehind(const Point & a, const Point & b, const RoadReference & ref);
  Point getLineEndForEM(int line_idx);
  Point walkAlongCenterline(int line_idx, const Point & from, double distance);
  double computeLaneWidth(int line_idx);

  // 动态 parking_lot
  geometry_msgs::msg::Polygon generateParkingParkingLot(
    const Point & target_pos, const RoadReference & ref);
  void publishDynamicParkingLot(const geometry_msgs::msg::Polygon & poly);

  // ★ 障碍物检测 + parking_lot 扩展
  void onObjects(const PredictedObjects::ConstSharedPtr msg);
  bool isPointInObstacleZone(const Point & pt, const PredictedObjects & objects);
  bool isPointInParkingLot(const Point & pt);
  void extendParkingLotToContain(const Point & target_pt, const RoadReference & ref);
  bool computeOutboundAstarGoal(const Point & tp);
  bool computeOutboundAstarGoalOnly(const Point & tp);  // 只算坐标不push任务

  // ★ 变道/掉头 parking_lot 生成
  geometry_msgs::msg::Polygon generateLaneChangeParkingLot(int from, int to);

  // 调度
  void processTargetVehicle(const Point & target_pos);
  void planReturnTrip();
  void addLaneChangeSequence(int from, int to, std::deque<PlanTask> & tasks);
  void executeNextTask();

  // ★★★ 虚线捷径决策 (Dashed-line Shortcut Decision) ★★★
  // routing graph: 算全局前进路径长度
  lanelet::ConstLanelet getClosestLanelet(const Point & pt);
  double computeRouteLength(const lanelet::ConstLanelet & from,
                            const lanelet::ConstLanelet & to);
  // 虚线信息
  struct DashedSeg {
    lanelet::Id way_id;              // 虚线way id
    Point p1, p2;                    // 虚线两端点
    lanelet::ConstLanelet lane_a;    // 虚线一侧车道
    lanelet::ConstLanelet lane_b;    // 虚线另一侧车道(对面)
  };
  std::vector<DashedSeg> dashed_segments_;
  void extractDashedSegments();      // 从地图提取6条虚线+两侧车道
  Point projectToLaneCenterline(const lanelet::ConstLanelet & ll, const Point & pt);

  // ★★★ 虚线掉头边增强路径搜索 (U-turn Edge Augmented Routing) ★★★
  struct AugEdge {
    int to; double cost; bool is_uturn; lanelet::Id via_dashed;
  };
  std::vector<lanelet::ConstLanelet> aug_lanelets_;   // 原始lanelet节点
  std::vector<std::vector<AugEdge>> aug_graph_;       // 邻接表
  void buildAugmentedGraph();
  int laneletIndexOf(const lanelet::ConstLanelet & ll);
  struct AugStep { int lanelet_idx; bool arrived_by_uturn; };
  std::vector<AugStep> augmentedSearch(const Point & from, const Point & to);
  void augmentedPathToTasks(const std::vector<AugStep> & path,
                            const Point & ego, const Point & final_target, bool is_return);

  // routing graph 成员
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  // 订阅
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr astar_completed_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr target_vehicle_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr matched_vehicle_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charge_complete_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr return_goal_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;  // 真实感知障碍物

  // 发布
  rclcpp::Publisher<StateMachine>::SharedPtr pub_state_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_astar_goal_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_empty_trajectory_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_em_goal_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_dynamic_parking_lot_;

  rclcpp::TimerBase::SharedPtr timer_;

  // 状态
  PlannerState current_state_ = PlannerState::IDLE;
  Odometry::ConstSharedPtr odom_;
  PoseStamped current_goal_;
  bool has_current_goal_ = false;
  TaskType current_task_type_ = TaskType::EM;

  std::deque<PlanTask> task_queue_;
  bool is_return_trip_ = false;
  bool return_trip_planned_ = false;
  bool near_road_end_pullout_ = false;
  std::string last_astar_description_;  // 最后执行的A*任务描述  // 充电完成A*直接走到道路尽头(目标车在路口)  // 返程路径是否已规划过
  bool is_em_segment_ = false;  // EM中间分段标记, true时用5m提前触发

  Point target_vehicle_pos_;
  bool has_target_vehicle_ = false;

  // ★ 多目标车调度 (scheduler每次发一辆, 充完反馈)
  void onTargetSequence(const PredictedObjects::ConstSharedPtr msg);
  void startNextTargetVehicle();             // 保留接口(多车模式由scheduler驱动)
  rclcpp::Subscription<PredictedObjects>::SharedPtr target_sequence_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_vehicle_served_;  // 充完一辆的反馈
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_arrived_station_; // 回站反馈
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_complete_sub_; // 全部结束信号
  void onTaskComplete(const std_msgs::msg::Bool::ConstSharedPtr msg);
  bool multi_return_to_station_ = false;     // 多车模式趟间回站(非最终)
  bool multi_vehicle_mode_ = false;          // 是否多车调度模式
  bool has_pending_target_ = false;          // 有暂存的目标车待处理(等地图/odom就绪)
  Point pending_target_pos_;                 // 暂存的目标车位置
  bool multi_pullout_to_next_ = false;       // 多车模式: A*驶离完成后发served去下一辆
  Point matched_vehicle_pos_;
  bool has_matched_vehicle_ = false;
  bool need_lane_change_outbound_ = false;

  PoseStamped return_final_goal_;
  bool has_return_goal_ = false;
  bool charge_completed_ = false;

  PoseStamped outbound_final_astar_;
  RoadReference road_ref_target_;
  geometry_msgs::msg::Polygon cached_parking_lot_;
  PredictedObjects::ConstSharedPtr perceived_objects_;  // 真实感知障碍物  // 缓存的 parking_lot

  std::vector<std::vector<Point>> all_centerlines_;
  std::vector<std::vector<lanelet::Id>> all_lanelet_ids_;  // 每个点对应的原始lanelet ID
  lanelet::LaneletMapPtr lanelet_map_ = nullptr;
  lanelet::ConstLanelets road_lanelets_;
  bool map_loaded_ = false;

  rclcpp::Time astar_start_time_;
  Point astar_start_pos_;
  bool astar_timeout_triggered_ = false;

  double switch_distance_threshold_;
  int park_side_;
  double em_distance_;
  double astar_distance_;
  double astar_timeout_sec_;
  double timeout_front_distance_;
  double timeout_move_threshold_;
  double return_astar_pullout_dist_;
  double return_em_offset_;
  // ★ 虚线捷径决策参数
  double uturn_shift_;              // 掉头横移估计(m)
  double uturn_lane_tol_;           // 判断车在可掉头车道的容差(m)
  // ★ 中心线合并参数
  double merge_dist_;               // 中心线合并距离阈值(m)
  double merge_cos_;                // 中心线合并方向余弦阈值
  double max_em_dist_;              // EM分段最大距离(m)
  double half_lane_width_;          // 半车道宽(m)
  double parking_margin_;           // parking余量(m)
  // ★ A*停车目标参数
  double astar_forward_dist_;       // A*停车前进距离(m)
  double astar_left_offset_;        // A*停车左偏(m)
  double astar_obstacle_extra_;     // A*障碍物避让额外距离(m)
  // ★ parking_lot 尺寸参数
  double plot_astar_front_;         // A*停车parking_lot前方(m)
  double plot_lanechange_front_;    // 变道parking_lot前方(m)
  double plot_lanechange_rear_;     // 变道parking_lot后方(m)
  double plot_uturn_front_;         // 掉头parking_lot前方(m)
  double plot_uturn_rear_;          // 掉头parking_lot后方(m)
  double plot_uturn_wleft_margin_;  // 掉头parking_lot左侧余量(m)
  double plot_uturn_wright_;        // 掉头parking_lot右侧(m)
};

} // namespace state_machine

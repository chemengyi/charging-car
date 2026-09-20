#include "task_planner/task_planner.hpp"

namespace state_machine
{

// ============================================================================
task_planner::task_planner(const rclcpp::NodeOptions & options)
: rclcpp::Node("task_planner", options)
{
  switch_distance_threshold_ = this->declare_parameter<double>("switch_distance_threshold", 0.5);
  park_side_                 = this->declare_parameter<int>("park_side", 0);
  em_distance_               = this->declare_parameter<double>("em_distance", 10.0);
  astar_distance_            = this->declare_parameter<double>("astar_distance", 6.0);
  timeout_move_threshold_    = this->declare_parameter<double>("timeout_move_threshold", 0.3);
  return_astar_pullout_dist_ = this->declare_parameter<double>("return_astar_pullout_dist", 4.0);
  return_em_offset_          = this->declare_parameter<double>("return_em_offset", 4.0);

  // ★ 虚线捷径决策参数
  uturn_shift_               = this->declare_parameter<double>("uturn_shift", 8.0);
  uturn_lane_tol_            = this->declare_parameter<double>("uturn_lane_tol", 0.5);
  // ★ 中心线合并参数
  merge_dist_                = this->declare_parameter<double>("merge_dist", 3.0);
  merge_cos_                 = this->declare_parameter<double>("merge_cos", 0.5);
  max_em_dist_               = this->declare_parameter<double>("max_em_dist", 1000.0);
  half_lane_width_           = this->declare_parameter<double>("half_lane_width", 2.0);
  parking_margin_            = this->declare_parameter<double>("parking_margin", 0.5);
  // ★ A*停车目标参数
  astar_forward_dist_        = this->declare_parameter<double>("astar_forward_dist", 4.0);
  astar_left_offset_         = this->declare_parameter<double>("astar_left_offset", 1.0);
  astar_obstacle_extra_      = this->declare_parameter<double>("astar_obstacle_extra", 6.0);
  // ★ parking_lot 尺寸参数
  plot_astar_front_          = this->declare_parameter<double>("plot_astar_front", 13.0);
  plot_lanechange_front_     = this->declare_parameter<double>("plot_lanechange_front", 7.0);
  plot_lanechange_rear_      = this->declare_parameter<double>("plot_lanechange_rear", 1.0);
  plot_uturn_front_          = this->declare_parameter<double>("plot_uturn_front", 8.0);
  plot_uturn_rear_           = this->declare_parameter<double>("plot_uturn_rear", 8.0);
  plot_uturn_wleft_margin_   = this->declare_parameter<double>("plot_uturn_wleft_margin", 3.0);
  plot_uturn_wright_         = this->declare_parameter<double>("plot_uturn_wright", 2.0);

  RCLCPP_INFO(get_logger(), "park_side=%d, em=%.1f, astar=%.1f, ret_pullout=%.1f, ret_offset=%.1f",
    park_side_, em_distance_, astar_distance_, return_astar_pullout_dist_, return_em_offset_);
  RCLCPP_INFO(get_logger(), "虚线捷径: uturn_shift=%.1f, merge_dist=%.1f, max_em_dist=%.1f",
    uturn_shift_, merge_dist_, max_em_dist_);

  odom_sub_ = create_subscription<Odometry>("/localization/kinematic_state", 100,
    std::bind(&task_planner::onOdometry, this, std::placeholders::_1));
  goal_sub_ = create_subscription<PoseStamped>("/planning/mission_planning/goal", 1,
    std::bind(&task_planner::onGoal, this, std::placeholders::_1));
  astar_completed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/planning/scenario_planning/task_completed", 1,
    std::bind(&task_planner::onAstarCompleted, this, std::placeholders::_1));
  target_vehicle_sub_ = create_subscription<PredictedObjects>("/task_planner/target_vehicle", 1,
    std::bind(&task_planner::onTargetVehicle, this, std::placeholders::_1));
  // ★ 多目标车调度序列 (scheduler每次发一辆, 充完反馈后再发下一辆)
  target_sequence_sub_ = create_subscription<PredictedObjects>(
    "/task_planner/target_vehicle_sequence", rclcpp::QoS(1).transient_local(),
    std::bind(&task_planner::onTargetSequence, this, std::placeholders::_1));
  pub_vehicle_served_ = create_publisher<std_msgs::msg::Bool>(
    "/task_planner/vehicle_served", 1);
  pub_arrived_station_ = create_publisher<std_msgs::msg::Bool>(
    "/task_planner/arrived_station", 1);
  task_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/task_planner/task_complete", rclcpp::QoS(1).transient_local(),
    std::bind(&task_planner::onTaskComplete, this, std::placeholders::_1));
  matched_vehicle_sub_ = create_subscription<PredictedObjects>(
    "/perception/object_recognition/objects_goal", 1,
    std::bind(&task_planner::onMatchedVehicle, this, std::placeholders::_1));
  map_sub_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&task_planner::onMap, this, std::placeholders::_1));
  charge_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/task_planner/charge_complete", 1,
    std::bind(&task_planner::onChargeComplete, this, std::placeholders::_1));
  return_goal_sub_ = create_subscription<PoseStamped>(
    "/task_planner/return_goal", 1,
    std::bind(&task_planner::onReturnGoal, this, std::placeholders::_1));

  // ★ 真实感知障碍物 (用于停车避障检测)
  objects_sub_ = create_subscription<PredictedObjects>(
    "/perception/object_recognition/objects", 1,
    std::bind(&task_planner::onObjects, this, std::placeholders::_1));

  pub_state_ = create_publisher<StateMachine>("/planning/task_planner/state", rclcpp::QoS(1).reliable());
  pub_astar_goal_ = create_publisher<PoseStamped>("/plannig/task_planner/goal", 1);
  pub_empty_trajectory_ = create_publisher<Trajectory>("/planning/scenario_planning/trajectory", 10);
  pub_em_goal_ = create_publisher<PoseStamped>("/readjson/goal", 1);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/task_planner/visualization", 1);
  pub_dynamic_parking_lot_ = create_publisher<geometry_msgs::msg::Polygon>(
    "/task_planner/dynamic_parking_lot", rclcpp::QoS(1).reliable().transient_local());

  const auto period_ns = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&task_planner::onTimer, this));

  RCLCPP_INFO(get_logger(), "task_planner v5 初始化完成 [去程+返程]");
}

// ============================================================================
//  工具函数
// ============================================================================
PoseStamped task_planner::makePose(double x, double y, double yaw) {
  PoseStamped ps; ps.header.frame_id = "map"; ps.header.stamp = this->now();
  ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = 0.0;
  tf2::Quaternion q; q.setRPY(0,0,yaw); q.normalize();
  ps.pose.orientation.x=q.x(); ps.pose.orientation.y=q.y(); ps.pose.orientation.z=q.z(); ps.pose.orientation.w=q.w();
  return ps;
}
void task_planner::clearAllGoals() {
  has_current_goal_ = false;
  // ★ 不清轨迹! 轨迹只在真正需要时清空 (EM到达切A*, 最终停车等)
}
void task_planner::sendEmGoal(const PoseStamped & g) {
  has_current_goal_ = false;
  current_goal_ = g; has_current_goal_ = true; current_task_type_ = TaskType::EM;
  pub_em_goal_->publish(g);
  RCLCPP_INFO(get_logger(), "  → EM目标: (%.2f,%.2f)", g.pose.position.x, g.pose.position.y);
}
void task_planner::clearEmGoal() {
}
void task_planner::sendAstarGoal(const PoseStamped & g) {
  has_current_goal_ = false;
  current_goal_ = g; has_current_goal_ = true; current_task_type_ = TaskType::ASTAR;
  pub_astar_goal_->publish(g);
  RCLCPP_INFO(get_logger(), "  → A*目标: (%.2f,%.2f)", g.pose.position.x, g.pose.position.y);
}
void task_planner::publishState(uint8_t v, const geometry_msgs::msg::Pose & p) {
  StateMachine m; m.state = v; m.pose = p; pub_state_->publish(m);
}
double task_planner::distanceTo(const PoseStamped & g) {
  if (!odom_) return 1e9;
  double dx = odom_->pose.pose.position.x - g.pose.position.x;
  double dy = odom_->pose.pose.position.y - g.pose.position.y;
  return std::sqrt(dx*dx+dy*dy);
}
bool task_planner::isDataReady() {
  if (!odom_) { RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"等odom..."); return false; }
  if (!map_loaded_) { RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"等地图..."); return false; }
  return true;
}

// ============================================================================
//  道路解析
// ============================================================================
void task_planner::extractRoadCenterlines() {
  all_centerlines_.clear();
  all_lanelet_ids_.clear();
  for (const auto & ll : road_lanelets_) {
    const auto & cl = ll.centerline(); if (cl.size()<2) continue;
    std::vector<Point> line;
    std::vector<lanelet::Id> ids;
    for (const auto & pt : cl) {
      Point p; p.x=pt.basicPoint().x(); p.y=pt.basicPoint().y(); p.z=pt.basicPoint().z();
      line.push_back(p);
      ids.push_back(ll.id());
    }
    all_centerlines_.push_back(line);
    all_lanelet_ids_.push_back(ids);
  }
  RCLCPP_INFO(get_logger(), "  原始中心线: %zu 条", all_centerlines_.size());

  // ★ 合并连续的中心线段 (同一条物理车道可能被分成多段lanelet)
  mergeCenterlines();
}

void task_planner::mergeCenterlines() {
  if(all_centerlines_.size() <= 1) return;
  const double MERGE_DIST = merge_dist_;   // 端点距离阈值
  const double MERGE_COS = merge_cos_;   // 方向余弦阈值(yaml)
  int n = (int)all_centerlines_.size();

  // 建立连接关系: next[i]=j 表示 line_i 的末端接到 line_j 的起点
  std::vector<int> nxt(n, -1), prv(n, -1);

  for(int i = 0; i < n; i++) {
    const auto & li = all_centerlines_[i];
    if(li.size() < 2) continue;
    // line_i 末端方向
    double di_x = li.back().x - li[li.size()-2].x;
    double di_y = li.back().y - li[li.size()-2].y;
    double di_len = std::sqrt(di_x*di_x + di_y*di_y);
    if(di_len < 0.01) continue;
    di_x /= di_len; di_y /= di_len;

    double best_d = MERGE_DIST; int best_j = -1;
    for(int j = 0; j < n; j++) {
      if(i == j) continue;
      const auto & lj = all_centerlines_[j];
      if(lj.size() < 2) continue;
      double dx = lj.front().x - li.back().x;
      double dy = lj.front().y - li.back().y;
      double dist = std::sqrt(dx*dx + dy*dy);
      if(dist >= best_d) continue;
      // 方向检查
      double dj_x = lj[1].x - lj[0].x;
      double dj_y = lj[1].y - lj[0].y;
      double dj_len = std::sqrt(dj_x*dj_x + dj_y*dj_y);
      if(dj_len < 0.01) continue;
      dj_x /= dj_len; dj_y /= dj_len;
      if(di_x*dj_x + di_y*dj_y > MERGE_COS) {
        best_d = dist; best_j = j;
      }
    }
    if(best_j >= 0 && prv[best_j] < 0) {
      nxt[i] = best_j; prv[best_j] = i;
    }
  }

  // 沿链合并
  std::vector<bool> visited(n, false);
  std::vector<std::vector<Point>> merged;
  std::vector<std::vector<lanelet::Id>> merged_ids;
  for(int i = 0; i < n; i++) {
    if(visited[i] || prv[i] >= 0) continue;
    std::vector<Point> chain;
    std::vector<lanelet::Id> chain_ids;
    for(int cur = i; cur >= 0; cur = nxt[cur]) {
      visited[cur] = true;
      const auto & seg = all_centerlines_[cur];
      const auto & seg_ids = all_lanelet_ids_[cur];
      if(!chain.empty()) {
        double dx = chain.back().x - seg.front().x;
        double dy = chain.back().y - seg.front().y;
        int start = (std::sqrt(dx*dx+dy*dy) < MERGE_DIST) ? 1 : 0;
        for(int k = start; k < (int)seg.size(); k++) {
          chain.push_back(seg[k]);
          chain_ids.push_back(seg_ids[k]);
        }
      } else {
        chain = seg;
        chain_ids = seg_ids;
      }
    }
    if(chain.size() >= 2) {
      merged.push_back(chain);
      merged_ids.push_back(chain_ids);
    }
  }
  for(int i = 0; i < n; i++)
    if(!visited[i] && all_centerlines_[i].size() >= 2) {
      merged.push_back(all_centerlines_[i]);
      merged_ids.push_back(all_lanelet_ids_[i]);
    }

  RCLCPP_INFO(get_logger(), "  中心线合并: %zu → %zu 条", all_centerlines_.size(), merged.size());
  all_centerlines_ = merged;
  all_lanelet_ids_ = merged_ids;
}

int task_planner::findNearestCenterlineIndex(const Point & pt) {
  double md=1e9; int best=-1;
  for (size_t i=0;i<all_centerlines_.size();++i)
    for (const auto & p : all_centerlines_[i]) {
      double d=std::sqrt(std::pow(p.x-pt.x,2)+std::pow(p.y-pt.y,2));
      if (d<md){md=d;best=(int)i;}
    }
  return best;
}

Point task_planner::findNearestPointOnLine(int idx, const Point & pt) {
  Point best; double md=1e9;
  for (const auto & p : all_centerlines_[idx]) {
    double d=std::sqrt(std::pow(p.x-pt.x,2)+std::pow(p.y-pt.y,2));
    if(d<md){md=d;best=p;}
  }
  return best;
}

int task_planner::findPointIndexOnLine(int line_idx, const Point & pt) {
  int best=0; double md=1e9;
  const auto & line = all_centerlines_[line_idx];
  for (size_t j=0;j<line.size();++j) {
    double d=std::sqrt(std::pow(line[j].x-pt.x,2)+std::pow(line[j].y-pt.y,2));
    if(d<md){md=d;best=(int)j;}
  }
  return best;
}

RoadReference task_planner::computeRoadRef(int idx, const Point & pt) {
  RoadReference ref;
  if (idx<0||idx>=(int)all_centerlines_.size()) return ref;
  const auto & line = all_centerlines_[idx];
  int bi=0; double md=1e9;
  for (size_t j=0;j<line.size();++j){double d=std::sqrt(std::pow(line[j].x-pt.x,2)+std::pow(line[j].y-pt.y,2));if(d<md){md=d;bi=(int)j;}}
  ref.nearest_point=line[bi]; ref.point_idx=bi; ref.line_idx=idx;
  const int N=2;  // ★ 小窗口, 弯道时更准确
  int si=std::max(0,bi-N), ei=std::min((int)line.size()-1,bi+N);
  if(si==ei){if(ei+1<(int)line.size())ei++;else if(si>0)si--;else return ref;}
  double dx=line[ei].x-line[si].x, dy=line[ei].y-line[si].y, len=std::sqrt(dx*dx+dy*dy);
  if(len<1e-6) return ref;
  ref.ref_dx=dx/len; ref.ref_dy=dy/len; ref.ref_yaw=std::atan2(ref.ref_dy,ref.ref_dx);
  ref.valid=true;
  return ref;
}

bool task_planner::isSameDirection(int la, int lb) {
  if (la<0||lb<0) return false;
  auto ra = computeRoadRef(la, all_centerlines_[la][all_centerlines_[la].size()/2]);
  auto rb = computeRoadRef(lb, all_centerlines_[lb][all_centerlines_[lb].size()/2]);
  if (!ra.valid||!rb.valid) return false;
  double dot = ra.ref_dx*rb.ref_dx + ra.ref_dy*rb.ref_dy;
  return dot > 0;  // >0 同向, <0 反向
}

int task_planner::findOppositeLane(int cur) {
  if (cur<0) return -1;
  for (int i=0;i<(int)all_centerlines_.size();++i) {
    if (i==cur) continue;
    if (!isSameDirection(cur, i)) return i;
  }
  return -1;
}

bool task_planner::isBehind(const Point & a, const Point & b, const RoadReference & ref) {
  // a在b后方 = a到b的投影在ref方向上为正
  double dx = b.x - a.x, dy = b.y - a.y;
  double proj = dx * ref.ref_dx + dy * ref.ref_dy;
  return proj > 0;  // 正: a在b后面(沿行驶方向)
}

// ============================================================================
//  获取中心线末端点 (往回退2m, 确保在lanelet内部, 用于EM目标)
// ============================================================================
Point task_planner::getLineEndForEM(int line_idx) {
  const auto & line = all_centerlines_[line_idx];
  // ★ 取道路中心线的倒数第二个实际点
  int idx = std::max(0, (int)line.size() - 2);
  return line[idx];
}

// ============================================================================
//  沿中心线走指定距离 (正=前进方向, 负=后退方向)
//  返回中心线上的点, 保证在道路上
// ============================================================================
Point task_planner::walkAlongCenterline(int line_idx, const Point & from, double distance) {
  const auto & line = all_centerlines_[line_idx];
  int start_idx = findPointIndexOnLine(line_idx, from);
  int step = (distance >= 0) ? 1 : -1;
  double remaining = std::abs(distance);
  int idx = start_idx;

  // ★ 沿中心线走, 返回实际的中心线点 (不插值)
  while(remaining > 0) {
    int next = idx + step;
    if(next < 0 || next >= (int)line.size()) break;
    double dx = line[next].x - line[idx].x;
    double dy = line[next].y - line[idx].y;
    double seg_len = std::sqrt(dx*dx + dy*dy);
    remaining -= seg_len;
    idx = next;
  }
  return line[idx];  // ★ 返回中心线上的实际点
}

// ============================================================================
//  回调
// ============================================================================
void task_planner::onOdometry(const Odometry::ConstSharedPtr msg) {
  odom_=msg;
  if(has_pending_target_ && map_loaded_ && odom_ && current_state_==PlannerState::IDLE) {
    has_pending_target_ = false;
    RCLCPP_INFO(get_logger(), "[onOdometry] odom就绪, 触发暂存目标车");
    processTargetVehicle(pending_target_pos_);
  }
}

void task_planner::onGoal(const PoseStamped::ConstSharedPtr msg) {
  RCLCPP_INFO(get_logger(),"[onGoal] rviz (%.2f,%.2f), EM目标由道路解析计算",msg->pose.position.x,msg->pose.position.y);
}

void task_planner::onMap(const HADMapBin::ConstSharedPtr msg) {
  RCLCPP_INFO(get_logger(),"[onMap] 收到lanelet地图...");
  lanelet_map_=std::make_shared<lanelet::LaneletMap>();
  // ★ 一次性构建 routing graph (和mission_planner一致)
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_, &traffic_rules_ptr_, &routing_graph_ptr_);
  road_lanelets_=lanelet::utils::query::laneletLayer(lanelet_map_);
  extractRoadCenterlines(); map_loaded_=true;
  RCLCPP_INFO(get_logger(),"[onMap] %zu 条中心线", all_centerlines_.size());

  // ★ 提取虚线段(6条)+两侧车道, 用于虚线捷径决策
  extractDashedSegments();
  RCLCPP_INFO(get_logger(),"[onMap] 提取到 %zu 条虚线捷径", dashed_segments_.size());

  // ★ 建虚线掉头边增强图 (U-turn Edge Augmented Routing)
  buildAugmentedGraph();

  if(has_pending_target_ && odom_ && current_state_==PlannerState::IDLE) {
    has_pending_target_ = false;
    RCLCPP_INFO(get_logger(), "[onMap] 地图就绪, 触发暂存目标车");
    processTargetVehicle(pending_target_pos_);
  }
}

void task_planner::onTargetVehicle(const PredictedObjects::ConstSharedPtr msg) {
  if (!msg||msg->objects.empty()) return;
  for (const auto & obj : msg->objects) {
    bool v=false; for(const auto & c:obj.classification) if(c.label>=1&&c.label<=4){v=true;break;}
    if(!v) continue;
    target_vehicle_pos_=obj.kinematics.initial_pose_with_covariance.pose.position;
    has_target_vehicle_=true;
    if(current_state_==PlannerState::IDLE&&map_loaded_&&odom_) processTargetVehicle(target_vehicle_pos_);
    return;
  }
}

// ★ 多目标车调度: scheduler每次只发一辆, 充完反馈后scheduler再发下一辆
void task_planner::onTargetSequence(const PredictedObjects::ConstSharedPtr msg) {
  if(!msg || msg->objects.empty()) return;
  if(current_state_ != PlannerState::IDLE) {
    RCLCPP_WARN(get_logger(), "[onTargetSequence] 非IDLE状态, 忽略");
    return;
  }
  Point tp = msg->objects[0].kinematics.initial_pose_with_covariance.pose.position;
  multi_vehicle_mode_ = true;
  RCLCPP_INFO(get_logger(), "======== [多车调度] 收到目标车 (%.2f, %.2f) ========", tp.x, tp.y);
  target_vehicle_pos_ = tp;
  has_target_vehicle_ = true;
  if(map_loaded_ && odom_) {
    processTargetVehicle(tp);
  } else {
    has_pending_target_ = true;
    pending_target_pos_ = tp;
    RCLCPP_WARN(get_logger(), "  地图/odom未就绪, 暂存目标车等待触发");
  }
}

// (保留接口, 多车模式由scheduler驱动)
void task_planner::startNextTargetVehicle() {
}

void task_planner::onMatchedVehicle(const PredictedObjects::ConstSharedPtr msg) {
  if(!msg||msg->objects.empty()||!map_loaded_) return;
  // 返程不处理车牌匹配
  if(is_return_trip_) return;
  // 变道中不处理
  if(need_lane_change_outbound_ &&
     current_state_!=PlannerState::OUTBOUND_EM &&
     current_state_!=PlannerState::OUTBOUND_ASTAR) {
    // 只在去程的最后EM/A*阶段才接受
  }
  // 同车道或变道完成后的阶段才接受
  const auto & obj=msg->objects[0];
  matched_vehicle_pos_=obj.kinematics.initial_pose_with_covariance.pose.position;
  has_matched_vehicle_=true;

  int ml=findNearestCenterlineIndex(matched_vehicle_pos_);
  RoadReference ref=computeRoadRef(ml,matched_vehicle_pos_);
  if(!ref.valid) return;

  bool fwd=(park_side_==1); double sign=fwd?1.0:-1.0;
  {
    double gx=matched_vehicle_pos_.x+sign*astar_distance_*ref.ref_dx;
    double gy=matched_vehicle_pos_.y+sign*astar_distance_*ref.ref_dy;
    Point gp; gp.x=gx; gp.y=gy; gp.z=0;
    // ★ 用目标车所在车道(ml), 不用findNearestCenterlineIndex
    auto gr=computeRoadRef(ml,findNearestPointOnLine(ml,gp));
    outbound_final_astar_=makePose(gx, gy, gr.valid?gr.ref_yaw:ref.ref_yaw);
  }

  RCLCPP_INFO(get_logger(),"[onMatchedVehicle] ★ (%.2f,%.2f)→A*更新(%.2f,%.2f)",
    matched_vehicle_pos_.x,matched_vehicle_pos_.y,
    outbound_final_astar_.pose.position.x,outbound_final_astar_.pose.position.y);
  publishMarkers();
}

void task_planner::onChargeComplete(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if(!msg->data) return;
  charge_completed_=true;
  RCLCPP_INFO(get_logger(),"[onChargeComplete] ★ 充电完成信号!");

  if(!odom_ || !map_loaded_) {
    RCLCPP_WARN(get_logger(),"  odom或地图未就绪, 等待...");
    current_state_ = PlannerState::CHARGING;
    return;
  }

  if(!has_target_vehicle_) {
    RCLCPP_ERROR(get_logger(),"  无目标车数据!");
    current_state_ = PlannerState::CHARGING;
    return;
  }

  int tgt_line = findNearestCenterlineIndex(target_vehicle_pos_);
  Point center_pt = findNearestPointOnLine(tgt_line, target_vehicle_pos_);
  auto ref = computeRoadRef(tgt_line, center_pt);
  if(!ref.valid) { RCLCPP_ERROR(get_logger(),"  参考向量无效!"); return; }

  // ★ 全面清空旧状态
  has_current_goal_ = false;
  clearEmGoal();
  pub_empty_trajectory_->publish(Trajectory());
  bool parked_in_front = isBehind(target_vehicle_pos_, odom_->pose.pose.position, ref);
  RCLCPP_INFO(get_logger(), "  车辆停在目标车%s", parked_in_front ? "前方 → 直接切EM" : "后方 → 需A*驶离");

  // ★★★ 多车模式: 充完这辆 → (若不在道路上先A*驶离) → 反馈scheduler ★★★
  if(multi_vehicle_mode_) {
    // ★ 多车模式: 不判在不在路上, 一律 A* 驶回车道中心线(投影+前进一点)
    //   保证下一辆决策时车在车道中心, 位置准确
    int ego_line_now = findNearestCenterlineIndex(odom_->pose.pose.position);
    Point proj = findNearestPointOnLine(ego_line_now, odom_->pose.pose.position);
    // 沿车道前进方向走一点
    Point pullout_pt = walkAlongCenterline(ego_line_now, proj, astar_forward_dist_);
    auto pull_ref = computeRoadRef(ego_line_now, pullout_pt);
    double pull_yaw = pull_ref.valid ? pull_ref.ref_yaw : 0.0;

    RCLCPP_INFO(get_logger(),
      "  ★ 多车模式: A*驶回车道中心 (%.2f,%.2f)", pullout_pt.x, pullout_pt.y);

    multi_pullout_to_next_ = true;
    is_return_trip_ = false;
    charge_completed_ = false;
    return_trip_planned_ = false;
    near_road_end_pullout_ = false;
    task_queue_.clear();

    // 发 A* 驶回目标
    PoseStamped pullout_goal = makePose(pullout_pt.x, pullout_pt.y, pull_yaw);
    current_goal_ = pullout_goal;
    has_current_goal_ = true;
    last_astar_description_ = "多车驶回车道中心";
    pub_astar_goal_->publish(pullout_goal);
    publishState(StateMachine::PARKING, pullout_goal.pose);
    astar_start_time_ = this->now();
    astar_start_pos_ = odom_->pose.pose.position;
    current_state_ = PlannerState::OUTBOUND_ASTAR;
    // A*到达后(onTimer里 multi_pullout_to_next_分支) → 发served → scheduler发下一辆
    return;
  } else {
    is_return_trip_ = true;
  }

  if(parked_in_front && !multi_vehicle_mode_) {
    // ★ 停在前方 → 直接切 EM, 不需要 A* 驶离
    // 因为前方没有目标车挡路, 直接上路
    if(has_return_goal_) {
      planReturnTrip();
    } else {
      current_state_ = PlannerState::CHARGING;
      RCLCPP_INFO(get_logger(), "  等待返程目标...");
    }
  } else {
    // ★ 停在后方 → 需要 A* 先驶离目标车区域
    double left_x = -ref.ref_dy;
    double left_y = ref.ref_dx;
    double forward_dist = astar_forward_dist_;

    // 循环检测: 前方有障碍物则继续往前6m
    double goal_x, goal_y;
    for(int attempt = 0; attempt < 10; ++attempt) {
      goal_x = center_pt.x + 1.0 * left_x + forward_dist * ref.ref_dx;
      goal_y = center_pt.y + 1.0 * left_y + forward_dist * ref.ref_dy;
      Point check_pt; check_pt.x = goal_x; check_pt.y = goal_y; check_pt.z = 0;
      if(!perceived_objects_ || perceived_objects_->objects.empty() ||
         !isPointInObstacleZone(check_pt, *perceived_objects_)) break;
      RCLCPP_WARN(get_logger(), "  A*目标(前进%.1fm)在障碍物区域! +6m", forward_dist);
      forward_dist += astar_obstacle_extra_;
    }

    // ★ 检查: A*目标是否距离道路尽头 < 5m 或者冲出道路
    Point road_end = getLineEndForEM(tgt_line);
    double dx_re = goal_x - road_end.x, dy_re = goal_y - road_end.y;
    double dist_to_road_end = std::sqrt(dx_re*dx_re + dy_re*dy_re);

    // ★ 检查是否冲出道路: A*目标超过道路最后一个点
    const auto & road_last = all_centerlines_[tgt_line].back();
    auto last_ref = computeRoadRef(tgt_line, road_last);
    double proj_beyond = 0;
    if(last_ref.valid) {
      proj_beyond = (goal_x - road_last.x) * last_ref.ref_dx +
                    (goal_y - road_last.y) * last_ref.ref_dy;
    }
    bool overshoot = (proj_beyond > 0);  // 目标在道路末端之后

    if(dist_to_road_end < 5.0 || overshoot) {
      RCLCPP_INFO(get_logger(),
        "  ★ A*目标%s (距尾%.1fm, 超出%.1fm), A*直接走到道路尾部",
        overshoot ? "冲出道路" : "距道路尾部<5m", dist_to_road_end, proj_beyond);

      // 检查道路尾部是否在停车parking_lot内, 不在则扩展
      if(!isPointInParkingLot(road_last)) {
        if(last_ref.valid) {
          extendParkingLotToContain(road_last, last_ref);
          RCLCPP_INFO(get_logger(), "  停车ParkingLot已扩展覆盖道路尾部");
        }
      }

      // A*目标直接改为道路最后一个点, 不切EM
      PoseStamped road_end_goal = makePose(road_last.x, road_last.y,
        last_ref.valid ? last_ref.ref_yaw : ref.ref_yaw);
      sendAstarGoal(road_end_goal);
      near_road_end_pullout_ = true;  // 标记: A*到达道路尽头后直接掉头
    } else {
      // 正常流程: A*到pullout目标
      PoseStamped pullout = makePose(goal_x, goal_y, ref.ref_yaw);
      Point lot_pt; lot_pt.x = goal_x; lot_pt.y = goal_y; lot_pt.z = 0;
      extendParkingLotToContain(lot_pt, ref);
      sendAstarGoal(pullout);
      near_road_end_pullout_ = false;
    }

    current_state_ = PlannerState::RETURN_ASTAR;
    publishState(StateMachine::PARKING, current_goal_.pose);
    astar_timeout_triggered_ = false;
    astar_start_time_ = this->now();
    astar_start_pos_ = odom_->pose.pose.position;

    RCLCPP_INFO(get_logger(), "  A*驶离: (%.2f,%.2f) near_end=%d",
      current_goal_.pose.position.x, current_goal_.pose.position.y, near_road_end_pullout_);
  }
  publishMarkers();
}

void task_planner::onReturnGoal(const PoseStamped::ConstSharedPtr msg) {
  // 已完成 → 忽略
  if(current_state_ == PlannerState::COMPLETED) return;
  // 已经在返程中 → 忽略 (防止重复触发)
  if(is_return_trip_ && current_state_ != PlannerState::CHARGING) return;

  return_final_goal_ = *msg;
  if(!has_return_goal_) {
    has_return_goal_ = true;
    RCLCPP_INFO(get_logger(),"[onReturnGoal] 返程最终位置 (%.2f,%.2f)",
      msg->pose.position.x, msg->pose.position.y);
  }

  // 如果已经充完电在等, 立刻开始返程 (只触发一次)
  if(charge_completed_ && current_state_==PlannerState::CHARGING && map_loaded_ && odom_) {
    planReturnTrip();
  }

  // ★ 多车模式: 收到回站目标(趟间换电池), 返程回站但不COMPLETED
  if(multi_vehicle_mode_ && current_state_==PlannerState::IDLE && map_loaded_ && odom_) {
    RCLCPP_INFO(get_logger(),"[onReturnGoal] ★ 多车趟间回站(换电池), 返程回站");
    is_return_trip_ = true;
    return_trip_planned_ = false;
    charge_completed_ = true;
    multi_return_to_station_ = true;   // ★ 标记趟间回站, 到站发arrived_station而非COMPLETED
    // 注意: 不清 multi_vehicle_mode_, 可能还有下一趟
    planReturnTrip();
  }
}

// ★ 收到全部任务结束信号 → COMPLETED
void task_planner::onTaskComplete(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if(!msg->data) return;
  RCLCPP_INFO(get_logger(),"[onTaskComplete] ★ 收到全部任务结束信号 → COMPLETED");
  pub_empty_trajectory_->publish(Trajectory());
  multi_vehicle_mode_ = false;
  multi_return_to_station_ = false;
  task_queue_.clear();
  current_state_ = PlannerState::COMPLETED;
}

void task_planner::onAstarCompleted(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if(!msg->data) return;

  // onTimer 已经用距离判断处理了 A* 完成
  // 这里作为备份: 如果 freespace 报完成但 onTimer 还没触发
  if(!has_current_goal_) {
    // onTimer 已经处理过了, 忽略
    return;
  }

  double d = has_current_goal_ ? distanceTo(current_goal_) : 0.0;
  RCLCPP_INFO(get_logger(),"[onAstarCompleted] freespace报完成 (距目标%.2fm)", d);

  // 如果距离还远, 忽略误报
  if(d > 1.0) {
    RCLCPP_WARN(get_logger(),
      "[onAstarCompleted] 距目标还有%.2fm > 1m, 忽略! 重新发A*目标", d);
    pub_astar_goal_->publish(current_goal_);
    return;
  }

  // 距离够近
  has_current_goal_ = false;

  if(!task_queue_.empty() && task_queue_.front().type == TaskType::EM) {
    RCLCPP_INFO(get_logger(), "[onAstarCompleted] → 切EM");
    executeNextTask();
  } else if(task_queue_.empty()) {
    // ★ 虚线停车到达 = 返程停车完成
    if(is_return_trip_ && last_astar_description_.find("虚线停车") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "[onAstarCompleted] ★ 返程虚线停车完成!");
      pub_empty_trajectory_->publish(Trajectory());
      if(multi_return_to_station_) {
        // 趟间回站: 发arrived_station, 回IDLE等下一趟
        multi_return_to_station_ = false;
        is_return_trip_ = false;
        return_trip_planned_ = false;
        std_msgs::msg::Bool arr; arr.data = true;
        pub_arrived_station_->publish(arr);
        current_state_ = PlannerState::IDLE;
        RCLCPP_INFO(get_logger(), "  ★ 趟间回站到达, 发arrived_station, 等下一趟");
      } else {
        current_state_ = PlannerState::COMPLETED;
      }
      return;
    }
    // ★ 返程起步 A* 完成: 只有尚未规划过返程才触发
    if(charge_completed_ && has_return_goal_ && is_return_trip_ && !return_trip_planned_) {
      RCLCPP_INFO(get_logger(), "[onAstarCompleted] 返程起步A*完成 → 返程规划");
      return_trip_planned_ = true;
      planReturnTrip();
    } else if(charge_completed_ && !has_return_goal_ && is_return_trip_ && !return_trip_planned_) {
      RCLCPP_INFO(get_logger(), "[onAstarCompleted] 返程起步完成, 等返程目标");
      current_state_ = PlannerState::CHARGING;
    } else {
      // ★★★ 最终停车 → 任务完成 ★★★
      RCLCPP_INFO(get_logger(), "[onAstarCompleted] ★ 最终停车 → 任务完成");
      pub_empty_trajectory_->publish(Trajectory());
      task_queue_.clear();
      if(is_return_trip_) {
        if(multi_return_to_station_) {
          // 趟间回站: 发arrived_station, 回IDLE等下一趟
          multi_return_to_station_ = false;
          is_return_trip_ = false;
          return_trip_planned_ = false;
          std_msgs::msg::Bool arr; arr.data = true;
          pub_arrived_station_->publish(arr);
          current_state_ = PlannerState::IDLE;
          RCLCPP_INFO(get_logger(), "======== 趟间回站到达, 发arrived_station, 等下一趟 ========");
        } else {
          current_state_ = PlannerState::COMPLETED;
          RCLCPP_INFO(get_logger(), "======== 返程完成! 一切停止 ========");
        }
      } else {
        current_state_ = PlannerState::CHARGING;
        RCLCPP_INFO(get_logger(), "======== 去程完成, 等待充电... ========");
      }
    }
  } else {
    executeNextTask();
  }
}

// ============================================================================
//  计算车道宽度 (从 lanelet 的 leftBound/rightBound)
// ============================================================================
double task_planner::computeLaneWidth(int line_idx) {
  if (line_idx < 0 || line_idx >= (int)road_lanelets_.size()) return 4.0;
  const auto & ll = road_lanelets_[line_idx];
  const auto & left = ll.leftBound();
  const auto & right = ll.rightBound();
  if (left.empty() || right.empty()) return 4.0;
  size_t mid = left.size() / 2;
  if (mid >= right.size()) mid = 0;
  double dx = left[mid].basicPoint().x() - right[mid].basicPoint().x();
  double dy = left[mid].basicPoint().y() - right[mid].basicPoint().y();
  double w = std::sqrt(dx*dx + dy*dy);
  return w > 0.5 ? w : 4.0;
}

// ============================================================================
//  ★ 生成停车型 parking_lot ★
//  以 A* 目标点为基准, 参考向量左侧, 包含一整个车道
//  前10m + 后12m = 总长22m
// ============================================================================
geometry_msgs::msg::Polygon task_planner::generateParkingParkingLot(
  const Point & target_pos, const RoadReference & ref)
{
  geometry_msgs::msg::Polygon poly;
  if (!ref.valid) return poly;

  int target_line = findNearestCenterlineIndex(target_pos);
  double lane_w = computeLaneWidth(target_line);

  // 行驶方向
  double fwd_x = ref.ref_dx, fwd_y = ref.ref_dy;
  // 垂直方向: 左侧 = (-fwd_y, fwd_x)
  double perp_x = -fwd_y, perp_y = fwd_x;

  double front = plot_astar_front_;  // A*停车parking_lot前方(yaml)
  double rear  = 10.0;  // 后方10m

  // 矩形在参考向量左侧, 包含一整个车道
  double right_w = 1.0;          // 右侧留1m
  double left_w = lane_w + 1.0;  // 车道宽 + 1m 余量

  geometry_msgs::msg::Point32 p;
  p.z = 0;

  // 右后
  p.x = target_pos.x - rear * fwd_x - right_w * perp_x;
  p.y = target_pos.y - rear * fwd_y - right_w * perp_y;
  poly.points.push_back(p);
  // 右前
  p.x = target_pos.x + front * fwd_x - right_w * perp_x;
  p.y = target_pos.y + front * fwd_y - right_w * perp_y;
  poly.points.push_back(p);
  // 左前
  p.x = target_pos.x + front * fwd_x + left_w * perp_x;
  p.y = target_pos.y + front * fwd_y + left_w * perp_y;
  poly.points.push_back(p);
  // 左后
  p.x = target_pos.x - rear * fwd_x + left_w * perp_x;
  p.y = target_pos.y - rear * fwd_y + left_w * perp_y;
  poly.points.push_back(p);

  RCLCPP_INFO(get_logger(),
    "  [ParkingLot] 目标(%.2f,%.2f) 前%.0fm后%.0fm 左%.1fm右%.1fm 车道宽%.1f",
    target_pos.x, target_pos.y, front, rear, left_w, right_w, lane_w);

  return poly;
}

// ============================================================================
//  发布动态 parking_lot
// ============================================================================
void task_planner::publishDynamicParkingLot(const geometry_msgs::msg::Polygon & poly) {
  cached_parking_lot_ = poly;  // ★ 缓存
  pub_dynamic_parking_lot_->publish(poly);
  RCLCPP_INFO(get_logger(), "  → 动态ParkingLot已发布 (%zu角点)", poly.points.size());
}

// ============================================================================
//  真实感知障碍物回调
// ============================================================================
void task_planner::onObjects(const PredictedObjects::ConstSharedPtr msg) {
  if(msg) perceived_objects_ = msg;
}

// ============================================================================
//  检查点是否在某个障碍物区域内 (用实际dimensions+位置+朝向, 安全余量0.5m)
// ============================================================================
bool task_planner::isPointInObstacleZone(const Point & pt, const PredictedObjects & objects) {
  const double MARGIN = parking_margin_;
  for (const auto & obj : objects.objects) {
    bool is_v = false;
    for (const auto & c : obj.classification) if(c.label>=1&&c.label<=4){is_v=true;break;}
    if(!is_v) continue;

    const auto & pose = obj.kinematics.initial_pose_with_covariance.pose;
    double ox = pose.position.x, oy = pose.position.y;
    double obj_yaw = tf2::getYaw(pose.orientation);

    // 障碍物半长/半宽 + 安全余量
    double half_l = obj.shape.dimensions.x / 2.0 + MARGIN;
    double half_w = obj.shape.dimensions.y / 2.0 + MARGIN;

    // 将检测点转到障碍物局部坐标系
    double dx = pt.x - ox, dy = pt.y - oy;
    double cos_y = std::cos(-obj_yaw), sin_y = std::sin(-obj_yaw);
    double local_x = dx * cos_y - dy * sin_y;
    double local_y = dx * sin_y + dy * cos_y;

    if(std::abs(local_x) < half_l && std::abs(local_y) < half_w) {
      RCLCPP_INFO(get_logger(), "  [障碍物检测] 点(%.2f,%.2f)在障碍物(%.2f,%.2f)区域内! (%.1fx%.1f+%.1fm余量)",
        pt.x, pt.y, ox, oy, obj.shape.dimensions.x, obj.shape.dimensions.y, MARGIN);
      return true;
    }
  }
  return false;
}

// ============================================================================
//  检查点是否在 cached_parking_lot_ 内
// ============================================================================
bool task_planner::isPointInParkingLot(const Point & pt) {
  if(cached_parking_lot_.points.size() < 3) return true;  // 没有lot就默认ok

  // 射线法判断点在多边形内
  int n = cached_parking_lot_.points.size();
  bool inside = false;
  for(int i = 0, j = n-1; i < n; j = i++) {
    double xi = cached_parking_lot_.points[i].x, yi = cached_parking_lot_.points[i].y;
    double xj = cached_parking_lot_.points[j].x, yj = cached_parking_lot_.points[j].y;
    if(((yi > pt.y) != (yj > pt.y)) &&
       (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

// ============================================================================
//  扩展 parking_lot 向前, 确保包含目标点
// ============================================================================
void task_planner::extendParkingLotToContain(const Point & target_pt, const RoadReference & ref) {
  if(cached_parking_lot_.points.size() != 4 || !ref.valid) return;
  if(isPointInParkingLot(target_pt)) return;  // 已包含

  // 计算目标点在参考向量方向上超出前方多少
  // 用点[1](右前)作为当前前方边界参考
  double front_x = cached_parking_lot_.points[1].x;
  double front_y = cached_parking_lot_.points[1].y;

  // 从当前前方边界到目标点的投影距离
  double dx = target_pt.x - front_x;
  double dy = target_pt.y - front_y;
  double proj = dx * ref.ref_dx + dy * ref.ref_dy;

  if(proj > 0) {
    // 需要往前延伸 proj + 3m 余量
    double extend = proj + 3.0;
    cached_parking_lot_.points[1].x += extend * ref.ref_dx;
    cached_parking_lot_.points[1].y += extend * ref.ref_dy;
    cached_parking_lot_.points[2].x += extend * ref.ref_dx;
    cached_parking_lot_.points[2].y += extend * ref.ref_dy;
    publishDynamicParkingLot(cached_parking_lot_);
    RCLCPP_INFO(get_logger(), "  [ParkingLot] 向前扩展%.1fm, 覆盖目标(%.2f,%.2f)",
      extend, target_pt.x, target_pt.y);
  }
}

// ============================================================================
//  计算去程 A* 停车目标 (含障碍物检测)
//  返回 true: 正常停车, false: 放弃停车(两侧都有障碍物)
// ============================================================================
bool task_planner::computeOutboundAstarGoal(const Point & tp) {
  bool fwd = (park_side_ == 1);
  double sign = fwd ? 1.0 : -1.0;

  // 默认停后方
  {
    double gx = tp.x + sign * astar_distance_ * road_ref_target_.ref_dx;
    double gy = tp.y + sign * astar_distance_ * road_ref_target_.ref_dy;
    // ★ yaw用目标车所在车道(tgt_line)的最近中心线点参考向量
    Point gp; gp.x=gx; gp.y=gy; gp.z=0;
    int gl = findNearestCenterlineIndex(tp);  // tp是目标车位置,在道路上
    Point gn=findNearestPointOnLine(gl,gp);
    auto gr=computeRoadRef(gl,gn);
    outbound_final_astar_ = makePose(gx, gy, gr.valid?gr.ref_yaw:road_ref_target_.ref_yaw);
  }

  if(perceived_objects_ && !perceived_objects_->objects.empty()) {
    Point astar_pt = outbound_final_astar_.pose.position;
    if(isPointInObstacleZone(astar_pt, *perceived_objects_)) {
      RCLCPP_WARN(get_logger(), "  A*目标(后方)在障碍物区域! 尝试前方%.1fm", astar_distance_);
      {
        double gx = tp.x - sign * astar_distance_ * road_ref_target_.ref_dx;
        double gy = tp.y - sign * astar_distance_ * road_ref_target_.ref_dy;
        Point gp; gp.x=gx; gp.y=gy; gp.z=0;
        int gl = findNearestCenterlineIndex(tp);
        Point gn=findNearestPointOnLine(gl,gp);
        auto gr=computeRoadRef(gl,gn);
        outbound_final_astar_ = makePose(gx, gy, gr.valid?gr.ref_yaw:road_ref_target_.ref_yaw);
      }
      astar_pt = outbound_final_astar_.pose.position;

      if(isPointInObstacleZone(astar_pt, *perceived_objects_)) {
        RCLCPP_ERROR(get_logger(), "  前方也在障碍物区域! 放弃停车");
        return false;
      }
    }
  }

  // ★ parking_lot 在 executeNextTask 执行时再生成
  task_queue_.push_back({TaskType::ASTAR, outbound_final_astar_, "去程A*停车"});
  RCLCPP_INFO(get_logger(), "  A*停车目标: (%.2f,%.2f)",
    outbound_final_astar_.pose.position.x, outbound_final_astar_.pose.position.y);
  return true;
}

// ★ 只计算充电A*目标坐标, 不push任务 (供情况A/B使用)
bool task_planner::computeOutboundAstarGoalOnly(const Point & tp) {
  bool fwd = (park_side_ == 1);
  double sign = fwd ? 1.0 : -1.0;
  double gx = tp.x + sign * astar_distance_ * road_ref_target_.ref_dx;
  double gy = tp.y + sign * astar_distance_ * road_ref_target_.ref_dy;
  Point gp; gp.x=gx; gp.y=gy; gp.z=0;
  int gl = findNearestCenterlineIndex(tp);
  Point gn=findNearestPointOnLine(gl,gp);
  auto gr=computeRoadRef(gl,gn);
  outbound_final_astar_ = makePose(gx, gy, gr.valid?gr.ref_yaw:road_ref_target_.ref_yaw);
  // 障碍物检查(翻转到前方)
  if(perceived_objects_ && !perceived_objects_->objects.empty()) {
    if(isPointInObstacleZone(outbound_final_astar_.pose.position, *perceived_objects_)) {
      double gx2 = tp.x - sign * astar_distance_ * road_ref_target_.ref_dx;
      double gy2 = tp.y - sign * astar_distance_ * road_ref_target_.ref_dy;
      Point gp2; gp2.x=gx2; gp2.y=gy2; gp2.z=0;
      auto gr2=computeRoadRef(gl,findNearestPointOnLine(gl,gp2));
      outbound_final_astar_ = makePose(gx2, gy2, gr2.valid?gr2.ref_yaw:road_ref_target_.ref_yaw);
      if(isPointInObstacleZone(outbound_final_astar_.pose.position, *perceived_objects_)) return false;
    }
  }
  return true;
}

// ============================================================================
//  去程虚线/提前变道 (覆盖所有需要变道的场景)
// ============================================================================
// ============================================================================
//  虚线检测 & 提前变道
// ============================================================================
bool task_planner::isDashedLeftBoundary(int line_idx, int point_idx) {
  if(line_idx<0||line_idx>=(int)all_lanelet_ids_.size()) return false;
  if(point_idx<0||point_idx>=(int)all_lanelet_ids_[line_idx].size()) return false;
  try {
    auto ll = lanelet_map_->laneletLayer.get(all_lanelet_ids_[line_idx][point_idx]);
    std::string st = ll.leftBound().attributeOr("subtype","");
    return st == "dashed";
  } catch(...) { return false; }
}

Point task_planner::projectPointOnCenterline(int line_idx, const Point & pt) {
  const auto & line = all_centerlines_[line_idx];
  double min_d=1e9; Point best; best.x=best.y=best.z=0;
  for(int i=0;i<(int)line.size()-1;i++){
    double ax=line[i].x,ay=line[i].y,bx=line[i+1].x,by=line[i+1].y;
    double abx=bx-ax,aby=by-ay,apx=pt.x-ax,apy=pt.y-ay;
    double ab2=abx*abx+aby*aby; if(ab2<1e-10) continue;
    double t=std::max(0.0,std::min(1.0,(apx*abx+apy*aby)/ab2));
    Point p; p.x=ax+t*abx; p.y=ay+t*aby; p.z=0;
    double d=std::hypot(pt.x-p.x,pt.y-p.y);
    if(d<min_d){min_d=d;best=p;}
  }
  return best;
}


// ============================================================================
//  返程虚线/提前变道
// ============================================================================


// ============================================================================
//  去程调度
// ============================================================================
// ============================================================================
//  虚线捷径决策 (Dashed-line Shortcut Decision)
// ============================================================================

// 点 → 最近lanelet (用lanelet2 query API, 和default_planner一致)
lanelet::ConstLanelet task_planner::getClosestLanelet(const Point & pt) {
  lanelet::ConstLanelet closest;
  geometry_msgs::msg::Pose p;
  p.position.x = pt.x; p.position.y = pt.y; p.position.z = 0;
  lanelet::Lanelet closest_mut;
  if(lanelet::utils::query::getClosestLanelet(road_lanelets_, p, &closest_mut)) {
    closest = closest_mut;
  }
  return closest;
}

// 点到lanelet中心线距离

// routing graph: from→to 是否连通

// routing graph: from→to 最短前进路径长度(累加各lanelet中心线长度)
double task_planner::computeRouteLength(const lanelet::ConstLanelet & from,
                                        const lanelet::ConstLanelet & to) {
  if(!routing_graph_ptr_) return 1e18;
  lanelet::Optional<lanelet::routing::Route> route =
    routing_graph_ptr_->getRoute(from, to, 0);
  if(!route) return 1e18;  // 不连通

  lanelet::routing::LaneletPath path = route->shortestPath();
  if(path.empty()) return 1e18;

  double len = 0.0;
  for(const auto & ll : path) {
    const auto cl = lanelet::utils::generateFineCenterline(ll, 1.0);
    for(size_t i=0;i+1<cl.size();i++) {
      double dx = cl[i+1].x()-cl[i].x();
      double dy = cl[i+1].y()-cl[i].y();
      len += std::hypot(dx, dy);
    }
  }
  return len;
}

// 投影点到lanelet中心线上最近点
Point task_planner::projectToLaneCenterline(const lanelet::ConstLanelet & ll, const Point & pt) {
  const auto cl = lanelet::utils::generateFineCenterline(ll, 1.0);
  Point best; best.x=pt.x; best.y=pt.y; best.z=0;
  double bd = 1e18;
  for(const auto & p : cl) {
    double d = std::hypot(p.x()-pt.x, p.y()-pt.y);
    if(d < bd) { bd=d; best.x=p.x(); best.y=p.y(); best.z=0; }
  }
  return best;
}

// 提取地图里的虚线段: 每条dashed way + 共享它的两个lanelet(互为对面)
void task_planner::extractDashedSegments() {
  dashed_segments_.clear();
  if(!lanelet_map_) return;

  // 遍历所有linestring, 找subtype=dashed的
  for(const auto & ls : lanelet_map_->lineStringLayer) {
    std::string subtype = ls.attributeOr("subtype", "");
    if(subtype != "dashed") continue;
    if(ls.size() < 2) continue;

    DashedSeg seg;
    seg.way_id = ls.id();
    // 虚线两端点
    seg.p1.x = ls.front().x(); seg.p1.y = ls.front().y(); seg.p1.z = 0;
    seg.p2.x = ls.back().x();  seg.p2.y = ls.back().y();  seg.p2.z = 0;

    // 找共享这条虚线way作为left边界的两个lanelet
    std::vector<lanelet::ConstLanelet> users;
    for(const auto & ll : road_lanelets_) {
      if(ll.leftBound().id() == ls.id()) users.push_back(ll);
    }
    if(users.size() != 2) continue;  // 必须恰好2个(互为对面)
    seg.lane_a = users[0];
    seg.lane_b = users[1];
    dashed_segments_.push_back(seg);
  }
}

// ★ 判定自车和目标是否分别在共享同一条虚线的两条车道上(一边一个)
//   是 → 返回true并填充opp_lane(目标侧车道), 用于前后判定前提

// ★ 找目标点最近车道对应的共享虚线对面车道

// 核心: 找最优虚线捷径
// ============================================================================
//  虚线掉头边增强路径搜索 (U-turn Edge Augmented Routing)
// ============================================================================

int task_planner::laneletIndexOf(const lanelet::ConstLanelet & ll) {
  for(size_t i=0;i<aug_lanelets_.size();i++)
    if(aug_lanelets_[i].id()==ll.id()) return (int)i;
  return -1;
}

// ── 以下为旧变道逻辑的占位实现(新增强搜索架构不再使用, 仅为链接通过) ──
geometry_msgs::msg::Polygon task_planner::generateLaneChangeParkingLot(int, int) {
  return geometry_msgs::msg::Polygon();  // 空多边形
}
void task_planner::addLaneChangeSequence(int, int, std::deque<PlanTask> &) {
  // 新架构用A*掉头, 不再生成变道序列
}

// 建增强图: 前进边(following) + 掉头边(6条虚线, 双向, 权重=uturn_shift)
void task_planner::buildAugmentedGraph() {
  aug_lanelets_.clear();
  aug_graph_.clear();
  if(!routing_graph_ptr_) return;

  // 节点 = 所有road lanelet
  for(const auto & ll : road_lanelets_) aug_lanelets_.push_back(ll);
  int n = aug_lanelets_.size();
  aug_graph_.resize(n);

  // 前进边: following, 权重=后继lanelet中心线长度
  for(int i=0;i<n;i++) {
    for(const auto & nxt : routing_graph_ptr_->following(aug_lanelets_[i])) {
      int j = laneletIndexOf(nxt);
      if(j<0) continue;
      double len=0; const auto cl=lanelet::utils::generateFineCenterline(nxt,1.0);
      for(size_t k=0;k+1<cl.size();k++)
        len += std::hypot(cl[k+1].x()-cl[k].x(), cl[k+1].y()-cl[k].y());
      aug_graph_[i].push_back({j, len, false, lanelet::InvalId});
    }
  }

  // 掉头边: 6条虚线, lane_a↔lane_b 双向, 权重=uturn_shift
  for(const auto & seg : dashed_segments_) {
    int ia = laneletIndexOf(seg.lane_a);
    int ib = laneletIndexOf(seg.lane_b);
    if(ia<0 || ib<0) continue;
    aug_graph_[ia].push_back({ib, uturn_shift_, true, seg.way_id});
    aug_graph_[ib].push_back({ia, uturn_shift_, true, seg.way_id});
  }
  RCLCPP_INFO(get_logger(),"[增强图] %d节点, %zu虚线掉头边", n, dashed_segments_.size());
}

// Dijkstra 在增强图上搜最短路
std::vector<task_planner::AugStep>
task_planner::augmentedSearch(const Point & from, const Point & to) {
  std::vector<AugStep> result;
  if(aug_graph_.empty()) return result;
  lanelet::ConstLanelet from_ll = getClosestLanelet(from);
  lanelet::ConstLanelet to_ll = getClosestLanelet(to);
  int s = laneletIndexOf(from_ll), t = laneletIndexOf(to_ll);
  if(s<0||t<0) return result;

  int n = aug_graph_.size();
  std::vector<double> dist(n, 1e18);
  std::vector<int> prev(n, -1);
  std::vector<bool> prev_uturn(n, false);
  std::vector<bool> visited(n, false);
  dist[s]=0;
  using PQ = std::pair<double,int>;
  std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;
  pq.push({0,s});
  while(!pq.empty()) {
    int u=pq.top().second; pq.pop();
    if(visited[u]) continue;
    visited[u]=true;
    if(u==t) break;
    for(const auto & e : aug_graph_[u]) {
      if(dist[u]+e.cost < dist[e.to]) {
        dist[e.to]=dist[u]+e.cost;
        prev[e.to]=u;
        prev_uturn[e.to]=e.is_uturn;
        pq.push({dist[e.to], e.to});
      }
    }
  }
  if(dist[t]>=1e17) return result;  // 不连通

  // 回溯
  std::vector<AugStep> rev;
  int cur=t;
  while(cur!=-1) {
    rev.push_back({cur, prev_uturn[cur]});
    cur=prev[cur];
  }
  std::reverse(rev.begin(), rev.end());
  return rev;
}

// 路径转任务队列: 连续前进段合并成1个EM, 掉头边转A*掉头
void task_planner::augmentedPathToTasks(const std::vector<AugStep> & path,
    const Point & ego, const Point & final_target, bool is_return) {
  // path[0]=起点(自车lanelet), 每步arrived_by_uturn标记该步是否掉头到达
  // 分段: 遍历, 遇到掉头边就切分
  std::string pfx = is_return ? "返程" : "去程";

  size_t i=1;  // path[0]是起点
  bool first_seg = true;
  while(i < path.size()) {
    if(path[i].arrived_by_uturn) {
      // 掉头边: path[i-1] --掉头--> path[i]
      // 掉头点:
      Point uturn_pt;
      if(first_seg && i==1) {
        // 自车起点车道掉头 → 原地掉头(当前位置)
        uturn_pt = ego;
        RCLCPP_INFO(get_logger(),"  [增强] 起点原地掉头");
      } else if(i == path.size()-1) {
        // ★ 最后掉头+停车: 掉头点=最终目标投影到掉头前车道L2(与EM目标一致)
        const auto & Lll = aug_lanelets_[path[i-1].lanelet_idx];
        uturn_pt = projectToLaneCenterline(Lll, final_target);
        RCLCPP_INFO(get_logger(),"  [增强] 最后掉头@lanelet%ld 投影点(%.2f,%.2f)",
          Lll.id(), uturn_pt.x, uturn_pt.y);
      } else {
        // 中间车道掉头 → 掉头车道(path[i-1])原始中心线第2个点
        const auto & Lll = aug_lanelets_[path[i-1].lanelet_idx];
        const auto cl = lanelet::utils::generateFineCenterline(Lll, 1.0);
        if(cl.size()>=2) { uturn_pt.x=cl[1].x(); uturn_pt.y=cl[1].y(); uturn_pt.z=0; }
        else if(!cl.empty()) { uturn_pt.x=cl[0].x(); uturn_pt.y=cl[0].y(); uturn_pt.z=0; }
        RCLCPP_INFO(get_logger(),"  [增强] 中间掉头@lanelet%ld 第2点(%.2f,%.2f)",
          Lll.id(), uturn_pt.x, uturn_pt.y);
      }
      // 掉头落点: 掉头点投影到掉头后车道(path[i])
      const auto & land_ll = aug_lanelets_[path[i].lanelet_idx];
      Point landing = projectToLaneCenterline(land_ll, uturn_pt);
      int land_line = findNearestCenterlineIndex(landing);
      auto lref = computeRoadRef(land_line, landing);
      task_queue_.push_back({TaskType::ASTAR, makePose(landing.x, landing.y,
        lref.valid?lref.ref_yaw:0), pfx+"A*掉头"});
      first_seg=false;
      i++;
    } else {
      // 前进边: 累积连续前进直到下一个掉头或结束
      size_t j=i;
      while(j<path.size() && !path[j].arrived_by_uturn) j++;
      // 前进段 path[i-1]→...→path[j-1], EM目标=下一动作起点
      // 下一动作: 若j<path.size()是掉头(EM目标=下个掉头点); 否则是停车(EM目标=接近点)
      Point em_target;
      if(j < path.size()) {
        // 后面是掉头
        const auto & Lll = aug_lanelets_[path[j-1].lanelet_idx];  // 掉头前车道L2
        // ★ 特例: 最后动作是"掉头+停车"(掉头是路径最后一步) →
        //   EM目标 = 最终目标点投影到掉头前车道L2的中心线(投影点)
        bool last_uturn_then_stop = (j == path.size()-1);
        if(last_uturn_then_stop) {
          em_target = projectToLaneCenterline(Lll, final_target);
          RCLCPP_INFO(get_logger(),
            "  [增强] 最后掉头+停车: EM目标=最终点投影到L2(%.2f,%.2f)",
            em_target.x, em_target.y);
        } else {
          // 中间掉头: EM目标 = 掉头车道第2点
          const auto cl = lanelet::utils::generateFineCenterline(Lll, 1.0);
          if(cl.size()>=2){em_target.x=cl[1].x();em_target.y=cl[1].y();em_target.z=0;}
          else if(!cl.empty()){em_target.x=cl[0].x();em_target.y=cl[0].y();em_target.z=0;}
        }
      } else {
        // 后面是停车(纯前进到目标): EM目标 = 目标车道上接近最终点的位置
        int tgt_line = findNearestCenterlineIndex(final_target);
        Point non = findNearestPointOnLine(tgt_line, final_target);
        bool fwd=(park_side_==1); double sign=fwd?1.0:-1.0;
        em_target = walkAlongCenterline(tgt_line, non, sign*em_distance_);
      }
      int em_line = findNearestCenterlineIndex(em_target);
      auto eref = computeRoadRef(em_line, em_target);
      task_queue_.push_back({TaskType::EM, makePose(em_target.x, em_target.y,
        eref.valid?eref.ref_yaw:0), pfx+"EM(前进段)"});
      i=j;
    }
  }
}


void task_planner::processTargetVehicle(const Point & tp) {
  RCLCPP_INFO(get_logger(),"======== processTargetVehicle (%.2f,%.2f) ========",tp.x,tp.y);

  // ★ 全面清空旧状态
  is_return_trip_=false;
  return_trip_planned_ = false;
  near_road_end_pullout_ = false;
  task_queue_.clear();
  has_current_goal_ = false;
  astar_timeout_triggered_ = false;
  has_matched_vehicle_ = false;
  charge_completed_ = false;
  clearEmGoal();
  pub_empty_trajectory_->publish(Trajectory());

  int tgt_line = findNearestCenterlineIndex(tp);
  road_ref_target_ = computeRoadRef(tgt_line, tp);
  if(!road_ref_target_.valid){RCLCPP_ERROR(get_logger(),"参考向量无效!");return;}

  Point ego_pos = odom_->pose.pose.position;

  // ★★★ 虚线掉头边增强路径搜索 (U-turn Edge Augmented Routing) ★★★
  auto path = augmentedSearch(ego_pos, tp);
  if(path.empty()) {
    RCLCPP_ERROR(get_logger(),"  ★ 增强搜索: 目标无法到达! 跳过去下一辆");
    if(multi_vehicle_mode_) {
      std_msgs::msg::Bool s; s.data=true; pub_vehicle_served_->publish(s);
      charge_completed_=false; is_return_trip_=false;
      task_queue_.clear(); current_state_=PlannerState::IDLE;
    }
    return;
  }
  RCLCPP_INFO(get_logger(),"  ★ 增强搜索成功: %zu个节点", path.size());

  // 路径转任务(连续前进合并成EM, 掉头转A*)
  augmentedPathToTasks(path, ego_pos, tp, false);

  // 末尾追加 A*充电停车
  if(!computeOutboundAstarGoalOnly(tp)) {
    RCLCPP_ERROR(get_logger(),"  充电目标在障碍物区, 去下一辆");
    if(multi_vehicle_mode_){ std_msgs::msg::Bool s; s.data=true;
      pub_vehicle_served_->publish(s); current_state_=PlannerState::IDLE; }
    return;
  }
  task_queue_.push_back({TaskType::ASTAR, outbound_final_astar_, "去程A*充电停车"});

  RCLCPP_INFO(get_logger(),"  任务队列: %zu 个任务", task_queue_.size());
  for(size_t k=0;k<task_queue_.size();k++)
    RCLCPP_INFO(get_logger(),"    [%zu] %s %s (%.2f,%.2f)", k,
      task_queue_[k].type==TaskType::EM?"EM":"A*", task_queue_[k].description.c_str(),
      task_queue_[k].goal.pose.position.x, task_queue_[k].goal.pose.position.y);
  executeNextTask();
}

void task_planner::planReturnTrip() {
  RCLCPP_INFO(get_logger(),"======== planReturnTrip ========");

  // ★ 全面清空旧状态
  is_return_trip_=true;
  task_queue_.clear();
  has_current_goal_=false;
  astar_timeout_triggered_ = false;
  clearEmGoal();
  pub_empty_trajectory_->publish(Trajectory());

  Point ego_pos = odom_->pose.pose.position;
  Point final_pos = return_final_goal_.pose.position;

  int final_line = findNearestCenterlineIndex(final_pos);
  Point final_on_line = findNearestPointOnLine(final_line, final_pos);
  auto final_ref = computeRoadRef(final_line, final_on_line);
  if(!final_ref.valid){RCLCPP_ERROR(get_logger(),"无法计算最终位置参考!");return;}

  double final_yaw = final_ref.valid ? final_ref.ref_yaw : 0.0;

  // ★★★ 虚线掉头边增强路径搜索 (返程, 同去程一套) ★★★
  auto path = augmentedSearch(ego_pos, final_pos);
  if(path.empty()) {
    RCLCPP_ERROR(get_logger(),"  返程★ 增强搜索: 站点无法到达!");
    return;
  }
  RCLCPP_INFO(get_logger(),"  返程★ 增强搜索成功: %zu个节点", path.size());

  augmentedPathToTasks(path, ego_pos, final_pos, true);

  // 末尾追加 A*最终停车
  PoseStamped final_astar = makePose(final_pos.x, final_pos.y, final_yaw);
  task_queue_.push_back({TaskType::ASTAR, final_astar, "返程A*最终停车"});

  RCLCPP_INFO(get_logger(),"  返程任务队列: %zu 个任务", task_queue_.size());
  for(size_t k=0;k<task_queue_.size();k++)
    RCLCPP_INFO(get_logger(),"    [%zu] %s %s (%.2f,%.2f)", k,
      task_queue_[k].type==TaskType::EM?"EM":"A*", task_queue_[k].description.c_str(),
      task_queue_[k].goal.pose.position.x, task_queue_[k].goal.pose.position.y);
  executeNextTask();
}

void task_planner::executeNextTask() {
  // 清空当前目标
  has_current_goal_ = false;

  if(task_queue_.empty()) {
    pub_empty_trajectory_->publish(Trajectory());  // ★ 清空轨迹
    if(is_return_trip_) {
      if(multi_return_to_station_) {
        // ★ 趟间回站: 发arrived_station, 回IDLE等下一趟(不COMPLETED)
        multi_return_to_station_ = false;
        is_return_trip_ = false;
        return_trip_planned_ = false;
        std_msgs::msg::Bool arr; arr.data = true;
        pub_arrived_station_->publish(arr);
        current_state_ = PlannerState::IDLE;
        RCLCPP_INFO(get_logger(),"======== 趟间回站到达, 发arrived_station, 等下一趟 ========");
      } else {
        current_state_ = PlannerState::COMPLETED;
        RCLCPP_INFO(get_logger(),"======== 所有任务完成! ========");
      }
    } else {
      // 去程完成 → 充电中
      current_state_ = PlannerState::CHARGING;
      RCLCPP_INFO(get_logger(),"======== 去程完成, 等待充电... ========");
    }
    return;
  }

  PlanTask task = task_queue_.front();
  task_queue_.pop_front();

  RCLCPP_INFO(get_logger(),"── 执行任务: %s %s ──",
    task.type==TaskType::EM?"EM":"A*", task.description.c_str());

  if(task.type == TaskType::EM) {
    // ★ EM开始时动态虚线检查 (跳过已经由虚线逻辑生成的EM任务)
    if(odom_ && !task_queue_.empty()
       && task.description.find("虚线") == std::string::npos
       && task.description.find("动态") == std::string::npos) {
      bool has_lc = false;
      for(const auto & t : task_queue_)
        if(t.description.find("变道") != std::string::npos) { has_lc=true; break; }

      if(has_lc) {
        Point ego_pos=odom_->pose.pose.position;
        int ego_line=findNearestCenterlineIndex(ego_pos);
        int ego_idx=findPointIndexOnLine(ego_line,ego_pos);

        // 确定最终目标位置
        Point astar_tgt;
        if(is_return_trip_ && has_return_goal_) {
          astar_tgt = return_final_goal_.pose.position;
        } else if(!is_return_trip_ && road_ref_target_.valid) {
          astar_tgt.x=outbound_final_astar_.pose.position.x;
          astar_tgt.y=outbound_final_astar_.pose.position.y;
          astar_tgt.z=0;
        } else {
          goto skip_dashed_check;
        }

        // ★ 如果目标已在当前车道上, 不需要虚线变道, 直接跳过
        {
          int target_on_line = findNearestCenterlineIndex(astar_tgt);
          if(target_on_line == ego_line) goto skip_dashed_check;
        }

        {
          int mi=findPointIndexOnLine(ego_line,astar_tgt);
          Point mp=all_centerlines_[ego_line][mi];
          bool ahead=(mi>ego_idx);
          bool dashed=isDashedLeftBoundary(ego_line,mi);

          // Case1: 匹配点在前方+虚线 → 跨虚线直接停车
          if(ahead && dashed) {
            RCLCPP_INFO(get_logger(),"  ★ [动态虚线] Case1: 跨虚线直接停车");
            task_queue_.clear();
            if(is_return_trip_) return_trip_planned_ = true;  // ★ 防止重复规划
            auto mr=computeRoadRef(ego_line,mp);
            PoseStamped dashed_park_goal;
            if(is_return_trip_) {
              // ★ 返程: 用目标点最近车道的参考向量重新计算yaw
              Point rp = return_final_goal_.pose.position;
              int rl = findNearestCenterlineIndex(rp);
              auto rr = computeRoadRef(rl, findNearestPointOnLine(rl, rp));
              dashed_park_goal = makePose(rp.x, rp.y, rr.valid?rr.ref_yaw:0);
              return_trip_planned_ = true;  // ★ 防止到达后再次调planReturnTrip
            } else {
              dashed_park_goal = outbound_final_astar_;
            }
            task_queue_.push_front({TaskType::ASTAR, dashed_park_goal,
              is_return_trip_?"返程A*虚线停车":"去程A*虚线停车"});
            task_queue_.push_front({TaskType::EM,makePose(mp.x,mp.y,
              mr.valid?mr.ref_yaw:0),
              is_return_trip_?"返程EM(动态虚线匹配)":"去程EM(动态虚线匹配)"});
            executeNextTask();
            return;
          }

          // Case1失败 → 尝试Case2: 找前方虚线提前变道
          if(ahead && !dashed) {
            // 需要找到目标所在的线
            int tgt_line=findNearestCenterlineIndex(astar_tgt);
            if(tgt_line!=ego_line) {
              int end_i=(int)all_centerlines_[ego_line].size()-1;
              int best=-1; double best_d=1e9;
              for(int i=ego_idx+1;i<=end_i;i++){
                if(!isDashedLeftBoundary(ego_line,i)) continue;
                double dot_chk=(all_centerlines_[ego_line][i].x-ego_pos.x)*
                  computeRoadRef(ego_line,ego_pos).ref_dx+
                  (all_centerlines_[ego_line][i].y-ego_pos.y)*
                  computeRoadRef(ego_line,ego_pos).ref_dy;
                if(dot_chk<=0) continue;
                double d=std::hypot(all_centerlines_[ego_line][i].x-ego_pos.x,
                                    all_centerlines_[ego_line][i].y-ego_pos.y);
                if(d<best_d){best_d=d;best=i;}
              }
              if(best>=0) {
                Point dpt=all_centerlines_[ego_line][best];
                Point proj=projectPointOnCenterline(tgt_line,dpt);
                // ★ 硬性条件: A*落点必须在目标匹配点后方 (点积)
                Point dm=findNearestPointOnLine(tgt_line,astar_tgt);
                auto dr2=computeRoadRef(tgt_line,dm);
                if(dr2.valid){
                  double dl=(proj.x-dm.x)*dr2.ref_dx+(proj.y-dm.y)*dr2.ref_dy;
                  if(dl>0){
                    RCLCPP_INFO(get_logger(),"  ★ [动态] 虚线变道取消: 落点在目标前方(%.2f)",dl);
                    goto skip_dashed_check;
                  }
                }
                RCLCPP_INFO(get_logger(),"  ★ [动态虚线] Case2: 提前变道");
                task_queue_.clear();
                if(is_return_trip_) return_trip_planned_ = true;
                auto dr=computeRoadRef(ego_line,dpt);
                auto tr=computeRoadRef(tgt_line,proj);
                double rx=tr.valid?tr.ref_dy:0,ry=tr.valid?-tr.ref_dx:0;

                // 提前变道后的后续任务
                if(is_return_trip_) {
                  Point fol=findNearestPointOnLine(tgt_line,astar_tgt);
                  Point rpt=walkAlongCenterline(tgt_line,fol,-return_em_offset_);
                  auto rr=computeRoadRef(tgt_line,rpt);
                  task_queue_.push_back({TaskType::EM,makePose(rpt.x,rpt.y,
                    rr.valid?rr.ref_yaw:0),"返程EM(动态变道后直达)"});
                } else {
                  bool fwd=(park_side_==1); double sign=fwd?1.0:-1.0;
                  Point nearest=findNearestPointOnLine(tgt_line,target_vehicle_pos_);
                  Point em_pt=walkAlongCenterline(tgt_line,nearest,sign*em_distance_);
                  auto er=computeRoadRef(tgt_line,em_pt);
                  task_queue_.push_back({TaskType::EM,makePose(em_pt.x,em_pt.y,
                    er.valid?er.ref_yaw:0),"去程EM(动态变道后接近)"});
                  if(!computeOutboundAstarGoal(target_vehicle_pos_)){
                    if(has_return_goal_) planReturnTrip();
                  }
                }
                // 变道任务
            task_queue_.push_front({TaskType::ASTAR, makePose(proj.x+rx,proj.y+ry,
                  tr.valid?tr.ref_yaw:0),
                  "A*虚线变道→线#"+std::to_string(ego_line)+"到#"+std::to_string(tgt_line)});
                task_queue_.push_front({TaskType::EM,makePose(dpt.x,dpt.y,
                  dr.valid?dr.ref_yaw:0),
                  is_return_trip_?"返程EM(动态提前变道)":"去程EM(动态提前变道)"});
                executeNextTask();
                return;
              }
            }
          }

          // Case2: 匹配点在后方 → 找前方虚线提前变道
          if(!ahead) {
            int tgt_line=findNearestCenterlineIndex(astar_tgt);
            if(tgt_line!=ego_line) {
              int end_i=(int)all_centerlines_[ego_line].size()-1;
              int best=-1; double best_d=1e9;
              for(int i=ego_idx+1;i<=end_i;i++){
                if(!isDashedLeftBoundary(ego_line,i)) continue;
                double dot_chk=(all_centerlines_[ego_line][i].x-ego_pos.x)*
                  computeRoadRef(ego_line,ego_pos).ref_dx+
                  (all_centerlines_[ego_line][i].y-ego_pos.y)*
                  computeRoadRef(ego_line,ego_pos).ref_dy;
                if(dot_chk<=0) continue;
                double d=std::hypot(all_centerlines_[ego_line][i].x-ego_pos.x,
                                    all_centerlines_[ego_line][i].y-ego_pos.y);
                if(d<best_d){best_d=d;best=i;}
              }
              if(best>=0) {
                Point dpt=all_centerlines_[ego_line][best];
                Point proj=projectPointOnCenterline(tgt_line,dpt);
                // ★ 硬性条件: A*落点必须在目标匹配点后方 (点积)
                Point dm2=findNearestPointOnLine(tgt_line,astar_tgt);
                auto dr3=computeRoadRef(tgt_line,dm2);
                if(dr3.valid){
                  double dl2=(proj.x-dm2.x)*dr3.ref_dx+(proj.y-dm2.y)*dr3.ref_dy;
                  if(dl2>0){
                    RCLCPP_INFO(get_logger(),"  ★ [动态] 虚线变道取消(后方): 落点在目标前方(%.2f)",dl2);
                    goto skip_dashed_check;
                  }
                }
                RCLCPP_INFO(get_logger(),"  ★ [动态虚线] Case2(后方): 提前变道");
                task_queue_.clear();
                if(is_return_trip_) return_trip_planned_ = true;
                auto dr=computeRoadRef(ego_line,dpt);
                auto tr=computeRoadRef(tgt_line,proj);
                double rx=tr.valid?tr.ref_dy:0,ry=tr.valid?-tr.ref_dx:0;
                // 变道后的后续 (动态检查会再处理)
                Point tgt_end=getLineEndForEM(tgt_line);
                auto te=computeRoadRef(tgt_line,tgt_end);
                task_queue_.push_back({TaskType::EM,makePose(tgt_end.x,tgt_end.y,
                  te.valid?te.ref_yaw:0),"EM(动态变道后行驶)"});
                // 默认变道回去的任务 (动态检查可能替换)
                addLaneChangeSequence(tgt_line,ego_line,task_queue_);

            task_queue_.push_front({TaskType::ASTAR, makePose(proj.x+rx,proj.y+ry,
                  tr.valid?tr.ref_yaw:0),
                  "A*虚线变道→线#"+std::to_string(ego_line)+"到#"+std::to_string(tgt_line)});
                task_queue_.push_front({TaskType::EM,makePose(dpt.x,dpt.y,
                  dr.valid?dr.ref_yaw:0),"EM(动态提前变道)"});
                executeNextTask();
                return;
              }
            }
          }
        }
      }
    }
    skip_dashed_check:

    // ★ 检查: 如果车辆已经在EM目标附近 (<1m), 跳过这个EM任务
    if(odom_) {
      double dx = odom_->pose.pose.position.x - task.goal.pose.position.x;
      double dy = odom_->pose.pose.position.y - task.goal.pose.position.y;
      double d = std::sqrt(dx*dx + dy*dy);
      if(d < 1.0) {
        RCLCPP_INFO(get_logger(), "  ★ 已在EM目标附近(%.2fm < 1m), 跳过", d);
        executeNextTask();
        return;
      }

      // ★ EM 长距离自动分段 (>1000m 沿中心线走, 不走直线)
      const double MAX_EM_DIST = max_em_dist_;
      if(d > MAX_EM_DIST) {
        task_queue_.push_front(task);  // 最终目标放回队列

        // ★ 沿道路中心线走1000m找中间点 (不是直线!)
        int ego_line = findNearestCenterlineIndex(odom_->pose.pose.position);
        int ego_idx = findPointIndexOnLine(ego_line, odom_->pose.pose.position);
        const auto & line = all_centerlines_[ego_line];

        // 沿中心线累积距离走1000m
        double accum = 0;
        int mid_idx = ego_idx;

        // 判断行驶方向: 目标在前方(idx增大)还是后方(idx减小)
        int target_idx = findPointIndexOnLine(ego_line, task.goal.pose.position);
        int step = (target_idx >= ego_idx) ? 1 : -1;

        for(int i = ego_idx; i >= 0 && i < (int)line.size() - 1; i += step) {
          int next = i + step;
          if(next < 0 || next >= (int)line.size()) break;
          double seg_dx = line[next].x - line[i].x;
          double seg_dy = line[next].y - line[i].y;
          double seg_len = std::sqrt(seg_dx*seg_dx + seg_dy*seg_dy);
          accum += seg_len;
          mid_idx = next;
          if(accum >= MAX_EM_DIST) break;
        }

        Point mid_pt = line[mid_idx];
        auto mid_ref = computeRoadRef(ego_line, mid_pt);
        double mid_yaw = mid_ref.valid ? mid_ref.ref_yaw : tf2::getYaw(task.goal.pose.orientation);

        PoseStamped mid_goal = makePose(mid_pt.x, mid_pt.y, mid_yaw);

        // A*→EM 切换
        if(current_state_ == PlannerState::OUTBOUND_ASTAR ||
           current_state_ == PlannerState::RETURN_ASTAR) {
          publishState(StateMachine::LANDRIVING, mid_goal.pose);
        }
        current_state_ = is_return_trip_ ? PlannerState::RETURN_EM : PlannerState::OUTBOUND_EM;
        publishState(StateMachine::LANDRIVING, mid_goal.pose);
        is_em_segment_ = true;
        sendEmGoal(mid_goal);
        RCLCPP_INFO(get_logger(), "  EM分段: 总距%.1fm, 沿中心线走%.1fm → (%.2f,%.2f)",
          d, accum, mid_pt.x, mid_pt.y);
        return;
      }
    }

    // A*→EM 切换
    if(current_state_ == PlannerState::OUTBOUND_ASTAR ||
       current_state_ == PlannerState::RETURN_ASTAR) {
      publishState(StateMachine::LANDRIVING, task.goal.pose);
    }

    current_state_ = is_return_trip_ ? PlannerState::RETURN_EM : PlannerState::OUTBOUND_EM;
    publishState(StateMachine::LANDRIVING, task.goal.pose);
    is_em_segment_ = false;  // 最终段用0.5m
    sendEmGoal(task.goal);
  } else {
    // ★ 记录最后一个A*任务描述 (用于虚线停车完成判断)
    last_astar_description_ = task.description;

    // ★ 检查: 如果车辆已经在A*目标附近 (<1m), 跳过这个A*任务
    if(odom_) {
      double dx = odom_->pose.pose.position.x - task.goal.pose.position.x;
      double dy = odom_->pose.pose.position.y - task.goal.pose.position.y;
      double d_a = std::sqrt(dx*dx + dy*dy);
      if(d_a < 1.0) {
        RCLCPP_INFO(get_logger(), "  ★ 已在A*目标附近(%.2fm < 1m), 跳过 %s", d_a, task.description.c_str());
        executeNextTask();
        return;
      }
    }

    // ★ A* 执行时生成对应的 parking_lot
    if(task.description.find("掉头") != std::string::npos && odom_) {
      // ★ 掉头 parking_lot: 覆盖当前车道到对面车道的掉头空间
      int cur_line = findNearestCenterlineIndex(odom_->pose.pose.position);
      auto ref = computeRoadRef(cur_line, odom_->pose.pose.position);
      if(ref.valid) {
        double cx = odom_->pose.pose.position.x, cy = odom_->pose.pose.position.y;
        // 掉头终点(对面车道)
        double tx = task.goal.pose.position.x, ty = task.goal.pose.position.y;
        // 横向宽度 = 当前位置到掉头终点的横向距离 + 余量
        double px = -ref.ref_dy, py = ref.ref_dx;  // 左法向
        double lat = std::abs((tx-cx)*px + (ty-cy)*py);  // 横向跨度
        double w_left = lat + plot_uturn_wleft_margin_;   // 覆盖到对面车道 + 余量
        double w_right = plot_uturn_wright_;
        double front = plot_uturn_front_, rear = plot_uturn_rear_;  // 掉头前后空间
        geometry_msgs::msg::Polygon lot; geometry_msgs::msg::Point32 p; p.z=0;
        p.x=cx-rear*ref.ref_dx-w_right*px; p.y=cy-rear*ref.ref_dy-w_right*py; lot.points.push_back(p);
        p.x=cx+front*ref.ref_dx-w_right*px; p.y=cy+front*ref.ref_dy-w_right*py; lot.points.push_back(p);
        p.x=cx+front*ref.ref_dx+w_left*px;  p.y=cy+front*ref.ref_dy+w_left*py;  lot.points.push_back(p);
        p.x=cx-rear*ref.ref_dx+w_left*px;  p.y=cy-rear*ref.ref_dy+w_left*py;  lot.points.push_back(p);
        publishDynamicParkingLot(lot);
        RCLCPP_INFO(get_logger(),"  [ParkingLot掉头] 前%.1f后%.1f 左%.1f右%.1f",
          front,rear,w_left,w_right);
      }
    } else if(task.description.find("虚线停车") != std::string::npos && odom_) {
      // ★ 虚线停车: 特殊 parking_lot (必须在"停车"之前检查!)
      int cur_line = findNearestCenterlineIndex(odom_->pose.pose.position);
      auto ref = computeRoadRef(cur_line, odom_->pose.pose.position);
      if(ref.valid) {
        Point proj = projectPointOnCenterline(cur_line, task.goal.pose.position);
        double dp = (odom_->pose.pose.position.x-proj.x)*ref.ref_dx +
                    (odom_->pose.pose.position.y-proj.y)*ref.ref_dy;
        double front = dp > 0 ? 6.0 : 8.0;
        double rear  = dp > 0 ? 8.0 : 1.0;
        double wx = task.goal.pose.position.x - proj.x;
        double wy = task.goal.pose.position.y - proj.y;
        double w_half = std::hypot(wx, wy) + 1.0;
        double px = -ref.ref_dy, py = ref.ref_dx;
        double cx = odom_->pose.pose.position.x, cy = odom_->pose.pose.position.y;
        geometry_msgs::msg::Polygon lot; geometry_msgs::msg::Point32 p; p.z=0;
        p.x=cx-rear*ref.ref_dx-w_half*px; p.y=cy-rear*ref.ref_dy-w_half*py; lot.points.push_back(p);
        p.x=cx+front*ref.ref_dx-w_half*px; p.y=cy+front*ref.ref_dy-w_half*py; lot.points.push_back(p);
        p.x=cx+front*ref.ref_dx+w_half*px; p.y=cy+front*ref.ref_dy+w_half*py; lot.points.push_back(p);
        p.x=cx-rear*ref.ref_dx+w_half*px; p.y=cy-rear*ref.ref_dy+w_half*py; lot.points.push_back(p);
        publishDynamicParkingLot(lot);
        RCLCPP_INFO(get_logger(),"  [ParkingLot虚线停车] 前%.1f后%.1f 宽%.1f",front,rear,w_half*2);
      }
    } else if(task.description.find("虚线变道") != std::string::npos && odom_) {
      // ★ 虚线变道: 特殊 parking_lot (必须在"变道"之前检查!)
      auto ref = computeRoadRef(findNearestCenterlineIndex(odom_->pose.pose.position),
                                odom_->pose.pose.position);
      if(ref.valid) {
        int tgt_l = findNearestCenterlineIndex(task.goal.pose.position);
        Point proj_tgt = projectPointOnCenterline(tgt_l, odom_->pose.pose.position);
        double link = std::hypot(proj_tgt.x-odom_->pose.pose.position.x,
                                 proj_tgt.y-odom_->pose.pose.position.y);
        double left_w = link + 2.0, right_w = 1.0;
        double px = -ref.ref_dy, py = ref.ref_dx;
        double cx = odom_->pose.pose.position.x, cy = odom_->pose.pose.position.y;
        geometry_msgs::msg::Polygon lot; geometry_msgs::msg::Point32 p; p.z=0;
        p.x=cx-1.0*ref.ref_dx-right_w*px; p.y=cy-1.0*ref.ref_dy-right_w*py; lot.points.push_back(p);
        p.x=cx+6.0*ref.ref_dx-right_w*px; p.y=cy+6.0*ref.ref_dy-right_w*py; lot.points.push_back(p);
        p.x=cx+6.0*ref.ref_dx+left_w*px;  p.y=cy+6.0*ref.ref_dy+left_w*py;  lot.points.push_back(p);
        p.x=cx-1.0*ref.ref_dx+left_w*px;  p.y=cy-1.0*ref.ref_dy+left_w*py;  lot.points.push_back(p);
        publishDynamicParkingLot(lot);
        RCLCPP_INFO(get_logger(),"  [ParkingLot虚线变道] 前6后1 左%.1f右%.1f",left_w,right_w);
      }
    } else if(task.description.find("变道") != std::string::npos) {
      // 变道 A*: 从描述中解析 from/to 线索引
      int from_idx = -1, to_idx = -1;
      auto pos1 = task.description.find("#");
      auto pos2 = task.description.find("#", pos1+1);
      if(pos1 != std::string::npos) from_idx = std::stoi(task.description.substr(pos1+1));
      if(pos2 != std::string::npos) to_idx = std::stoi(task.description.substr(pos2+1));
      if(from_idx >= 0 && to_idx >= 0) {
        auto lot = generateLaneChangeParkingLot(from_idx, to_idx);
        if(!lot.points.empty()) publishDynamicParkingLot(lot);
      }
    } else if(task.description.find("停车") != std::string::npos) {
      // 普通停车 A*
      int line_idx = findNearestCenterlineIndex(task.goal.pose.position);
      Point nearest = findNearestPointOnLine(line_idx, task.goal.pose.position);
      auto ref = computeRoadRef(line_idx, nearest);
      if(ref.valid) {
        auto lot = generateParkingParkingLot(task.goal.pose.position, ref);
        if(!lot.points.empty()) publishDynamicParkingLot(lot);
      }
    } else if(task.description.find("线末端") != std::string::npos) {
      // ★ A*到道路末端: 生成覆盖到最后一个点的 parking_lot
      int line_idx = findNearestCenterlineIndex(task.goal.pose.position);
      auto ref = computeRoadRef(line_idx, task.goal.pose.position);
      if(ref.valid) {
        const double HALF_LANE_W = half_lane_width_;
        double total_w = 2*(4.0 + HALF_LANE_W * 2)/5;  // 与变道宽度一致
        double half_w = total_w / 2.0;
        double fwd_x = ref.ref_dx, fwd_y = ref.ref_dy;
        double perp_x = -fwd_y, perp_y = fwd_x;
        // 以当前车辆位置为基准, 向前5m起步, 检查是否覆盖末端点
        Point ego_pt = odom_ ? odom_->pose.pose.position : task.goal.pose.position;
        double front = 6.0;
        const auto & last_pt = all_centerlines_[line_idx].back();
        // 循环扩展直到覆盖 last_pt
        for(int i = 0; i < 10; i++) {
          geometry_msgs::msg::Polygon lot;
          geometry_msgs::msg::Point32 p; p.z = 0;
          double rear = 1.0;
          p.x=ego_pt.x-rear*fwd_x-half_w*perp_x; p.y=ego_pt.y-rear*fwd_y-half_w*perp_y; lot.points.push_back(p);
          p.x=ego_pt.x+front*fwd_x-half_w*perp_x; p.y=ego_pt.y+front*fwd_y-half_w*perp_y; lot.points.push_back(p);
          p.x=ego_pt.x+front*fwd_x+half_w*perp_x; p.y=ego_pt.y+front*fwd_y+half_w*perp_y; lot.points.push_back(p);
          p.x=ego_pt.x-rear*fwd_x+half_w*perp_x; p.y=ego_pt.y-rear*fwd_y+half_w*perp_y; lot.points.push_back(p);
          // 检查 last_pt 是否在内
          cached_parking_lot_ = lot;
          if(isPointInParkingLot(last_pt)) {
            publishDynamicParkingLot(lot);
            RCLCPP_INFO(get_logger(), "  [ParkingLot线末端] 前%.1fm, 宽%.1fm", front, total_w);
            break;
          }
          front += 2.0;  // 不够就继续向前扩展2m
        }
      }
    }

    // A* 任务: sendAstarGoal 内部会先 clearEmGoal + clearAllGoals
    current_state_ = is_return_trip_ ? PlannerState::RETURN_ASTAR : PlannerState::OUTBOUND_ASTAR;
    astar_timeout_triggered_ = false;
    astar_start_time_ = this->now();
    if(odom_) astar_start_pos_ = odom_->pose.pose.position;
    publishState(StateMachine::PARKING, task.goal.pose);
    sendAstarGoal(task.goal);
  }
}

// ============================================================================
//  主定时器 (10Hz)
// ============================================================================
void task_planner::onTimer() {
  if(!isDataReady()) return;

  switch(current_state_) {
    case PlannerState::IDLE:
    case PlannerState::CHARGING:
    case PlannerState::COMPLETED:
      break;

    // ── EM 阶段 (去程/返程通用) ──
    case PlannerState::OUTBOUND_EM:
    case PlannerState::RETURN_EM:
    {
      if(!has_current_goal_) break;
      publishState(StateMachine::LANDRIVING, current_goal_.pose);

      double d = distanceTo(current_goal_);
      // 中间段用5m提前触发(不减速), 最终段用0.5m
      double trigger = is_em_segment_ ? 5.0 : switch_distance_threshold_;

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
        "[%s] 距目标: %.3f m (触发: %.1f%s)",
        current_state_==PlannerState::OUTBOUND_EM?"EM_OUT":"EM_RET",
        d, trigger, is_em_segment_ ? " 中间段" : "");

      if(d < trigger) {
        RCLCPP_INFO(get_logger(),"[EM] ★ %s!", is_em_segment_ ? "中间段切换" : "到达");
        has_current_goal_ = false;
        is_em_segment_ = false;
        // ★ 最终段到达才清轨迹, 中间段不清(保持速度)
        if(d < switch_distance_threshold_) {
          clearEmGoal();
        }
        executeNextTask();
      }
      break;
    }

    // ── A* 阶段 (去程/返程通用) ──
    case PlannerState::OUTBOUND_ASTAR:
    case PlannerState::RETURN_ASTAR:
    {
      if(!has_current_goal_) break;
      publishState(StateMachine::PARKING, current_goal_.pose);

      double d = distanceTo(current_goal_);
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "[%s] 距目标: %.2f m",
        current_state_==PlannerState::OUTBOUND_ASTAR?"A*_OUT":"A*_RET", d);

      // ★ 去程 A* 停车: 持续检测障碍物, 但只在目标变更时重发一次
      if(current_state_ == PlannerState::OUTBOUND_ASTAR && !is_return_trip_ &&
         perceived_objects_ && !perceived_objects_->objects.empty()) {
        Point cur_pt = current_goal_.pose.position;
        if(isPointInObstacleZone(cur_pt, *perceived_objects_)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[A*] 当前停车目标在障碍物区域! 尝试更换...");
          bool fwd = (park_side_ == 1);
          double sign = fwd ? 1.0 : -1.0;
          // 尝试前方
          double new_x = target_vehicle_pos_.x - sign * astar_distance_ * road_ref_target_.ref_dx;
          double new_y = target_vehicle_pos_.y - sign * astar_distance_ * road_ref_target_.ref_dy;
          Point front_pt; front_pt.x = new_x; front_pt.y = new_y; front_pt.z = 0;

          if(isPointInObstacleZone(front_pt, *perceived_objects_)) {
            // 前方也有障碍物 → 放弃, 直接返程
            RCLCPP_ERROR(get_logger(), "[A*] ★ 前后都有障碍物! 放弃停车, 直接返程");
            has_current_goal_ = false;
            task_queue_.clear();
            pub_empty_trajectory_->publish(Trajectory());
            publishState(StateMachine::LANDRIVING, current_goal_.pose);
            if(has_return_goal_) {
              charge_completed_ = true;
              is_return_trip_ = true;
              planReturnTrip();
            } else {
              current_state_ = PlannerState::COMPLETED;
            }
            break;
          } else {
            // 更换到前方, 清空旧目标再发新的(只发一次)
            PoseStamped new_goal = makePose(new_x, new_y, road_ref_target_.ref_yaw);
            if(std::abs(new_goal.pose.position.x - current_goal_.pose.position.x) > 0.1 ||
               std::abs(new_goal.pose.position.y - current_goal_.pose.position.y) > 0.1) {
              has_current_goal_ = false;  // ★ 清空旧目标
              current_goal_ = new_goal;
              has_current_goal_ = true;
              outbound_final_astar_ = new_goal;
              extendParkingLotToContain(front_pt, road_ref_target_);
              pub_astar_goal_->publish(current_goal_);  // 只发一次
              RCLCPP_INFO(get_logger(), "[A*] 停车目标更换为前方: (%.2f,%.2f)", new_x, new_y);
              astar_start_time_ = this->now();
              if(odom_) astar_start_pos_ = odom_->pose.pose.position;
            }
          }
        }
      }

      // ★ 车辆到达 A* 目标点
      if(d < switch_distance_threshold_) {
        has_current_goal_ = false;

        // ★ A*→A* 连续: 只更新目标点, 不切换状态
        if(!task_queue_.empty() && task_queue_.front().type == TaskType::ASTAR) {
          PlanTask next = task_queue_.front();
          task_queue_.pop_front();
          RCLCPP_INFO(get_logger(), "[A*] ★ A*→A* 连续! 不切状态, 更新目标 → %s (%.2f,%.2f)",
            next.description.c_str(), next.goal.pose.position.x, next.goal.pose.position.y);

          // 生成对应的 parking_lot (虚线优先!)
          if(next.description.find("虚线停车") != std::string::npos && odom_) {
            // 虚线停车 parking_lot (与executeNextTask中一致)
            int cl = findNearestCenterlineIndex(odom_->pose.pose.position);
            auto rf = computeRoadRef(cl, odom_->pose.pose.position);
            if(rf.valid) {
              Point prj = projectPointOnCenterline(cl, next.goal.pose.position);
              double dp=(odom_->pose.pose.position.x-prj.x)*rf.ref_dx+
                        (odom_->pose.pose.position.y-prj.y)*rf.ref_dy;
              double ft=dp>0?6.0:8.0, rr=dp>0?8.0:1.0;
              double wh=std::hypot(next.goal.pose.position.x-prj.x,next.goal.pose.position.y-prj.y)+1.0;
              double ppx=-rf.ref_dy,ppy=rf.ref_dx;
              double cx=odom_->pose.pose.position.x,cy=odom_->pose.pose.position.y;
              geometry_msgs::msg::Polygon lot; geometry_msgs::msg::Point32 p; p.z=0;
              p.x=cx-rr*rf.ref_dx-wh*ppx;p.y=cy-rr*rf.ref_dy-wh*ppy;lot.points.push_back(p);
              p.x=cx+ft*rf.ref_dx-wh*ppx;p.y=cy+ft*rf.ref_dy-wh*ppy;lot.points.push_back(p);
              p.x=cx+ft*rf.ref_dx+wh*ppx;p.y=cy+ft*rf.ref_dy+wh*ppy;lot.points.push_back(p);
              p.x=cx-rr*rf.ref_dx+wh*ppx;p.y=cy-rr*rf.ref_dy+wh*ppy;lot.points.push_back(p);
              publishDynamicParkingLot(lot);
            }
          } else if(next.description.find("虚线变道") != std::string::npos && odom_) {
            auto rf=computeRoadRef(findNearestCenterlineIndex(odom_->pose.pose.position),odom_->pose.pose.position);
            if(rf.valid) {
              int tl=findNearestCenterlineIndex(next.goal.pose.position);
              Point pt=projectPointOnCenterline(tl,odom_->pose.pose.position);
              double lk=std::hypot(pt.x-odom_->pose.pose.position.x,pt.y-odom_->pose.pose.position.y);
              double lw=lk+2.0,rw=1.0,ppx=-rf.ref_dy,ppy=rf.ref_dx;
              double cx=odom_->pose.pose.position.x,cy=odom_->pose.pose.position.y;
              geometry_msgs::msg::Polygon lot; geometry_msgs::msg::Point32 p; p.z=0;
              p.x=cx-1.0*rf.ref_dx-rw*ppx;p.y=cy-1.0*rf.ref_dy-rw*ppy;lot.points.push_back(p);
              p.x=cx+6.0*rf.ref_dx-rw*ppx;p.y=cy+6.0*rf.ref_dy-rw*ppy;lot.points.push_back(p);
              p.x=cx+6.0*rf.ref_dx+lw*ppx;p.y=cy+6.0*rf.ref_dy+lw*ppy;lot.points.push_back(p);
              p.x=cx-1.0*rf.ref_dx+lw*ppx;p.y=cy-1.0*rf.ref_dy+lw*ppy;lot.points.push_back(p);
              publishDynamicParkingLot(lot);
            }
          } else if(next.description.find("变道") != std::string::npos) {
            int from_idx = -1, to_idx = -1;
            auto p1 = next.description.find("#");
            auto p2 = next.description.find("#", p1+1);
            if(p1 != std::string::npos) from_idx = std::stoi(next.description.substr(p1+1));
            if(p2 != std::string::npos) to_idx = std::stoi(next.description.substr(p2+1));
            if(from_idx >= 0 && to_idx >= 0) {
              auto lot = generateLaneChangeParkingLot(from_idx, to_idx);
              if(!lot.points.empty()) publishDynamicParkingLot(lot);
            }
          } else if(next.description.find("停车") != std::string::npos) {
            int nl = findNearestCenterlineIndex(next.goal.pose.position);
            Point nn = findNearestPointOnLine(nl, next.goal.pose.position);
            auto nref = computeRoadRef(nl, nn);
            if(nref.valid) {
              auto lot = generateParkingParkingLot(next.goal.pose.position, nref);
              if(!lot.points.empty()) publishDynamicParkingLot(lot);
            }
          }

          // 直接更新 A* 目标, 不切换状态
          current_goal_ = next.goal;
          has_current_goal_ = true;
          pub_astar_goal_->publish(next.goal);
          publishState(StateMachine::PARKING, next.goal.pose);
          astar_start_time_ = this->now();
          if(odom_) astar_start_pos_ = odom_->pose.pose.position;
        }
        // A*→EM 切换
        else if(!task_queue_.empty() && task_queue_.front().type == TaskType::EM) {
          RCLCPP_INFO(get_logger(), "[A*] ★ 变道到达! 距目标 %.3fm → 切EM", d);
          executeNextTask();
        }
        else if(task_queue_.empty()) {
          RCLCPP_INFO(get_logger(), "[A*] 队列空: charge=%d, return_goal=%d, trip=%d, planned=%d, near_end=%d",
            charge_completed_, has_return_goal_, is_return_trip_, return_trip_planned_, near_road_end_pullout_);

          // ★ 虚线停车A*到达且队列空 = 最终停车完成
          if(is_return_trip_ && last_astar_description_.find("虚线停车") != std::string::npos) {
            RCLCPP_INFO(get_logger(), "[A*] ★ 返程虚线停车完成!");
            pub_empty_trajectory_->publish(Trajectory());
            task_queue_.clear();
            current_state_ = PlannerState::COMPLETED;
            RCLCPP_INFO(get_logger(), "======== 返程完成! ========");
            break;
          }

          // ★ 近路口: 到达道路末端后直接用A*变道 (返程或多车衔接都需要)
          if(near_road_end_pullout_ && (is_return_trip_ || multi_pullout_to_next_) && odom_) {
            int cur_line = findNearestCenterlineIndex(odom_->pose.pose.position);
            int opp = findOppositeLane(cur_line);
            if(opp >= 0) {
              const auto & end_cur = all_centerlines_[cur_line].back();
              Point b_near = findNearestPointOnLine(opp, end_cur);
              auto ref_opp = computeRoadRef(opp, b_near);
              double yaw_opp = ref_opp.valid ? ref_opp.ref_yaw : 0.0;
              double rx = ref_opp.valid ? ref_opp.ref_dy : 0.0;
              double ry = ref_opp.valid ? -ref_opp.ref_dx : 0.0;

              // 生成变道 parking_lot
              auto lot = generateLaneChangeParkingLot(cur_line, opp);
              if(!lot.points.empty()) publishDynamicParkingLot(lot);

              // A*→A* 直接变道
              PoseStamped lc_goal = makePose(b_near.x + rx, b_near.y + ry, yaw_opp);
              current_goal_ = lc_goal;
              has_current_goal_ = true;
              pub_astar_goal_->publish(lc_goal);
              publishState(StateMachine::PARKING, lc_goal.pose);
              astar_start_time_ = this->now();
              astar_start_pos_ = odom_->pose.pose.position;
              near_road_end_pullout_ = false;  // 变道后恢复正常
              RCLCPP_INFO(get_logger(), "[A*] ★ 近路口→直接A*变道 (%.2f,%.2f)",
                lc_goal.pose.position.x, lc_goal.pose.position.y);
            } else {
              RCLCPP_WARN(get_logger(), "[A*] 找不到反向车道!");
              near_road_end_pullout_ = false;
            }
          }
          // ★ 多车模式: A*驶离回道路完成 → 发served → scheduler发下一辆
          else if(multi_pullout_to_next_ && odom_) {
            RCLCPP_INFO(get_logger(), "[A*] ★ 多车驶离完成, 反馈scheduler去下一辆");
            multi_pullout_to_next_ = false;
            std_msgs::msg::Bool served; served.data = true;
            pub_vehicle_served_->publish(served);
            charge_completed_ = false; is_return_trip_ = false;
            return_trip_planned_ = false; near_road_end_pullout_ = false;
            task_queue_.clear();
            current_state_ = PlannerState::IDLE;
            break;
          }
          // 正常返程起步
          else if(charge_completed_ && has_return_goal_ && is_return_trip_ && !return_trip_planned_) {
            RCLCPP_INFO(get_logger(), "[A*] ★ 返程起步到达! → 开始返程路径规划");
            return_trip_planned_ = true;
            planReturnTrip();
          } else if(charge_completed_ && !has_return_goal_ && is_return_trip_ && !return_trip_planned_) {
            RCLCPP_INFO(get_logger(), "[A*] ★ 返程起步到达! 等返程目标...");
            current_state_ = PlannerState::CHARGING;
          } else {
            RCLCPP_INFO(get_logger(), "[A*] ★ 最终停车到达! %.3fm → 完成", d);
            pub_empty_trajectory_->publish(Trajectory());
            task_queue_.clear();
            if(is_return_trip_) {
              current_state_ = PlannerState::COMPLETED;
              RCLCPP_INFO(get_logger(), "======== 返程完成! ========");
            } else {
              current_state_ = PlannerState::CHARGING;
              RCLCPP_INFO(get_logger(), "======== 去程完成, 等待充电... ========");
            }
          }
        }
        else {
          RCLCPP_INFO(get_logger(), "[A*] ★ A*到达! 距目标 %.3fm → 执行下一任务", d);
          executeNextTask();
        }
        break;
      }

      // ★ 超时检测: 15s 不动 → 自动返程 (去程停车阶段)
      if(odom_) {
        double elapsed = (this->now()-astar_start_time_).seconds();
        double mx=odom_->pose.pose.position.x-astar_start_pos_.x;
        double my=odom_->pose.pose.position.y-astar_start_pos_.y;
        double moved=std::sqrt(mx*mx+my*my);

        if(!astar_timeout_triggered_ && elapsed > 1500.0 && moved < timeout_move_threshold_ &&
           current_state_ == PlannerState::OUTBOUND_ASTAR && !is_return_trip_) {
          RCLCPP_WARN(get_logger(),
            "[A*] ★ 去程停车1500s未移动! 自动放弃, 直接返程");
          astar_timeout_triggered_ = true;
          has_current_goal_ = false;
          task_queue_.clear();
          pub_empty_trajectory_->publish(Trajectory());
          publishState(StateMachine::LANDRIVING, current_goal_.pose);
          charge_completed_ = true;
          is_return_trip_ = true;
          if(has_return_goal_) {
            planReturnTrip();
          } else {
            current_state_ = PlannerState::CHARGING;
            RCLCPP_INFO(get_logger(), "  等待返程目标...");
          }
          break;
        }
      }
      break;
    }
  }

  static int mc=0; if(++mc>=10){mc=0;publishMarkers();}
}

// ============================================================================
//  可视化
// ============================================================================
void task_planner::publishMarkers() {
  visualization_msgs::msg::MarkerArray ma;
  auto stamp=this->now(); int id=0;

  auto addM=[&](const Point&p,float r,float g,float b,const std::string&ns,const std::string&lbl,int shape){
    visualization_msgs::msg::Marker m;
    m.header.frame_id="map";m.header.stamp=stamp;m.ns=ns;m.id=id++;
    m.type=shape;m.action=0;m.pose.position=p;m.pose.position.z=0.5;m.pose.orientation.w=1;
    m.scale.x=shape==2?0.8:2.0;m.scale.y=shape==2?0.8:1.0;m.scale.z=shape==2?0.8:1.0;
    m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=0.7;
    m.lifetime=rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(m);
    visualization_msgs::msg::Marker t;t.header=m.header;t.ns=ns+"_t";t.id=id++;
    t.type=9;t.action=0;t.pose.position=p;t.pose.position.z=2;t.scale.z=0.4;
    t.color.r=r;t.color.g=g;t.color.b=b;t.color.a=1;t.text=lbl;
    t.lifetime=rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(t);
  };

  if(has_target_vehicle_) addM(target_vehicle_pos_,1,0,0,"tgt","TARGET CAR",1);
  if(has_matched_vehicle_) addM(matched_vehicle_pos_,1,0.5,0,"mtch","MATCHED",1);
  if(has_current_goal_) {
    Point gp=current_goal_.pose.position;
    if(current_task_type_==TaskType::EM) addM(gp,0,1,0,"cur","CURRENT EM",2);
    else addM(gp,0,0,1,"cur","CURRENT A*",2);
  }
  if(has_return_goal_) {
    // ★ RETURN GOAL: 用不同形状(CUBE=1)、不同颜色(黄色)、不同高度(z=3)区分
    Point rp = return_final_goal_.pose.position;
    visualization_msgs::msg::Marker rm;
    rm.header.frame_id="map"; rm.header.stamp=stamp; rm.ns="ret"; rm.id=id++;
    rm.type=1; rm.action=0; rm.pose.position=rp; rm.pose.position.z=3.0; rm.pose.orientation.w=1;
    rm.scale.x=1.0; rm.scale.y=1.0; rm.scale.z=1.0;
    rm.color.r=1; rm.color.g=1; rm.color.b=0; rm.color.a=0.7;
    rm.lifetime=rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(rm);
    visualization_msgs::msg::Marker rt;
    rt.header=rm.header; rt.ns="ret_t"; rt.id=id++;
    rt.type=9; rt.action=0; rt.pose.position=rp; rt.pose.position.z=5.0; rt.scale.z=0.6;
    rt.color.r=1; rt.color.g=1; rt.color.b=0; rt.color.a=1; rt.text="RETURN GOAL";
    rt.lifetime=rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(rt);
  }

  if(!ma.markers.empty()) pub_markers_->publish(ma);
}

} // namespace state_machine
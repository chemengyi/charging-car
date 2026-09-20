#include "task_planner/charge_scheduler.hpp"

namespace state_machine
{

// ============================================================================
//  构造
// ============================================================================
ChargeScheduler::ChargeScheduler(const rclcpp::NodeOptions & options)
: Node("charge_scheduler", options)
{
  uturn_shift_ = declare_parameter<double>("uturn_shift", 8.0);
  dashed_sample_step_ = declare_parameter<double>("dashed_sample_step", 2.0);

  // ABS相邻反向车道识别：共享边界ID优先；失败时采用几何共边和局部法向回退。
  abstract_opposite_angle_deg_ = declare_parameter<double>(
    "abstract_opposite_angle_deg", 25.0);
  abstract_normal_angle_deg_ = declare_parameter<double>(
    "abstract_normal_angle_deg", 30.0);
  abstract_search_radius_m_ = declare_parameter<double>(
    "abstract_search_radius_m", 15.0);
  abstract_min_lateral_m_ = declare_parameter<double>(
    "abstract_min_lateral_m", 1.0);
  abstract_max_lateral_m_ = declare_parameter<double>(
    "abstract_max_lateral_m", 12.0);
  abstract_max_longitudinal_m_ = declare_parameter<double>(
    "abstract_max_longitudinal_m", 3.0);
  abstract_parallel_span_m_ = declare_parameter<double>(
    "abstract_parallel_span_m", 4.0);
  abstract_boundary_gap_tol_m_ = declare_parameter<double>(
    "abstract_boundary_gap_tol_m", 0.50);
  abstract_boundary_min_samples_ = declare_parameter<int>(
    "abstract_boundary_min_samples", 3);

  distance_mode_      = declare_parameter<std::string>("distance_mode", "maneuver"); // ★ maneuver(ECD) | dsp | euclid | abstract
  objective_order_    = declare_parameter<std::string>("objective_order", "nlate_first"); // nlate_first | tover_first
  // ── 闭环重调度 ──
  discharge_rate_        = declare_parameter<double>("discharge_rate_pct_per_min", 0.5); // 兼容保留(逐车 ρ 优先)
  heavy_soc_threshold_   = declare_parameter<double>("heavy_soc_threshold", 20.0);   // ★ 重载电量下限(%)
  downgrade_mode_        = declare_parameter<std::string>("downgrade_mode", "light");// ★ 跌破后降级工况
  park_soc_threshold_    = declare_parameter<double>("park_soc_threshold", 5.0);   // ★ 停机等待阈值(%)
  num_chargers_          = declare_parameter<int>("num_chargers", 2);               // ★ 1=串行, 2=交替接力
  min_depart_soc_        = declare_parameter<double>("min_depart_soc", 0.5);        // ★ 接力最低出发电量
  battery_swap_          = declare_parameter<bool>("battery_swap", true);           // ★ 换电模式
  swap_time_min_         = declare_parameter<double>("swap_time_min", 10.0);        // ★ 换电耗时(min)
  enable_reschedule_     = declare_parameter<bool>("enable_reschedule", true);
  reschedule_threshold_  = declare_parameter<double>("reschedule_threshold_min", 10.0);

  // ── 多车事件驱动闭环实验（上层离散事件仿真）──
  execution_simulation_mode_ = declare_parameter<bool>("execution_simulation_mode", false);
  execution_compare_open_closed_ = declare_parameter<bool>("execution_compare_open_closed", true);
  execution_reschedule_policy_ = declare_parameter<std::string>(
    "execution_reschedule_policy", "global");
  disturbance_type_ = declare_parameter<std::string>("disturbance_type", "none");
  disturbance_delay_min_ = declare_parameter<double>("disturbance_delay_min", 0.0);
  disturbance_target_quantiles_ = declare_parameter<std::vector<double>>(
    "disturbance_target_quantiles", std::vector<double>{0.25, 0.50});
  disturbance_station_ranks_ = declare_parameter<std::vector<int64_t>>(
    "disturbance_station_ranks", std::vector<int64_t>{1, 2});
  reschedule_alns_iters_ = declare_parameter<int>("reschedule_alns_iters", 4000);
  account_reschedule_compute_time_ = declare_parameter<bool>(
    "account_reschedule_compute_time", false);

  // 目标优先级统一入口。OBJ-N 保持既有实验口径；OBJ-T 用于目标偏好敏感性。
  {
    std::string obj = objective_order_;
    std::transform(obj.begin(), obj.end(), obj.begin(),
      [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    if(obj=="tover_first" || obj=="tovertime_first" || obj=="t_over_first" ||
       obj=="tover,nlate,ctotal" || obj=="t") {
      objective_order_ = "tover_first";
    } else if(obj=="nlate_first" || obj=="n_late_first" ||
              obj=="nlate,tover,ctotal" || obj=="n") {
      objective_order_ = "nlate_first";
    } else {
      RCLCPP_WARN(get_logger(),
        "[目标顺序] 未知 objective_order=%s，回退为 nlate_first", obj.c_str());
      objective_order_ = "nlate_first";
    }
    RCLCPP_INFO(get_logger(), "[目标顺序] objective_order=%s", objectiveOrderLabel().c_str());
  }

  // ── 真实/RViz反馈兼容接口：旧Bool继续可用，新String为未来多车扩展 ──
  execution_feedback_topic_ = declare_parameter<std::string>(
    "execution_feedback_topic", "/task_planner/execution_event");
  assignment_meta_topic_ = declare_parameter<std::string>(
    "assignment_meta_topic", "/charge_scheduler/assignment_meta");
  prefer_structured_feedback_ = declare_parameter<bool>("prefer_structured_feedback", false);
  // ── 启发式参数（多种子实验用） ──
  heuristic_method_ = declare_parameter<std::string>("heuristic_method", "alns");
  alns_seed_        = declare_parameter<int>("alns_seed", 42);
  alns_iters_       = declare_parameter<int>("alns_iters", 3000);
  alns_seg_         = declare_parameter<int>("alns_segment", 100);
  alns_T0_ratio_    = declare_parameter<double>("alns_T0_ratio", 0.05);
  alns_cooling_     = declare_parameter<double>("alns_cooling", 0.9995);
  // ── HGS 参数（Vidal 2022, C&OR 140:105643, Table 1）──
  hgs_mu_       = declare_parameter<int>("hgs_mu", 25);          // 最小种群
  hgs_lambda_   = declare_parameter<int>("hgs_lambda", 40);      // 代规模
  hgs_nElite_   = declare_parameter<int>("hgs_n_elite", 4);      // 精英数
  hgs_nClosest_ = declare_parameter<int>("hgs_n_closest", 5);    // 多样性近邻数
  hgs_gamma_    = declare_parameter<int>("hgs_granular", 20);    // 粒度（近邻限制）
  hgs_nIt_      = declare_parameter<int>("hgs_n_it", 3000);      // 无改进迭代上限
  hgs_xiRef_    = declare_parameter<double>("hgs_xi_ref", 0.2);  // 目标可行比例  // ★ 与下层 task_planner 同步
  charging_station_.x = declare_parameter<double>("charging_station.x", 0.0);
  charging_station_.y = declare_parameter<double>("charging_station.y", 0.0);
  charging_station_.z = 0.0;

  avg_speed_                 = declare_parameter<double>("avg_speed", 66.7);               // m/min (4km/h)
  alpha_                     = declare_parameter<double>("alpha", 2.0);                    // τ=α·b
  alpha_scale_               = declare_parameter<double>("alpha_scale", 1.0);              // ★ α 缩放
  force_heuristic_      = declare_parameter<bool>("force_heuristic", false);  // ★ true=小规模也强制走启发式(M≤8 gap对照用)

  // ★★★ 能量感知调度模型参数 (一整块电池, kWh) ★★★
  battery_capacity_kwh_    = declare_parameter<double>("battery_capacity_kwh", 31.3);
  charge_power_kw_         = declare_parameter<double>("charge_power_kw", 100.0);
  charge_efficiency_       = declare_parameter<double>("charge_efficiency", 0.97);
  drive_consume_kwh_per_km_= declare_parameter<double>("drive_consume_kwh_per_km", 0.6);
  station_charge_power_kw_ = declare_parameter<double>("station_charge_power_kw", 120.0);
  soc_low_threshold_       = declare_parameter<double>("soc_low_threshold", 0.20);
  soc_depart_threshold_    = declare_parameter<double>("soc_depart_threshold", 0.80);
  target_charge_target_    = declare_parameter<double>("target_charge_target", 0.80);
  w_distance_              = declare_parameter<double>("weight_distance", 3.0);
  w_battery_               = declare_parameter<double>("weight_battery", 1.0);
  w_demand_                = declare_parameter<double>("weight_demand", 1.0);
  w_urgency_               = declare_parameter<double>("weight_urgency", 20.0);   // ★ 紧迫度(主导)

  auto xs = declare_parameter<std::vector<double>>("target_x", std::vector<double>{});
  auto ys = declare_parameter<std::vector<double>>("target_y", std::vector<double>{});
  auto bs = declare_parameter<std::vector<double>>("target_battery", std::vector<double>{});
  auto caps = declare_parameter<std::vector<double>>("target_capacity_kwh", std::vector<double>{});
  auto types = declare_parameter<std::vector<std::string>>("target_type", std::vector<std::string>{});
  auto modes = declare_parameter<std::vector<std::string>>("target_mode", std::vector<std::string>{});

  size_t n = std::min({xs.size(), ys.size(), bs.size()});
  for(size_t i=0;i<n;i++) {
    TargetVehicle tv;
    tv.pos.x = xs[i]; tv.pos.y = ys[i]; tv.pos.z = 0.0;
    tv.remaining_battery = bs[i];
    tv.capacity_kwh = (i < caps.size()) ? caps[i] : 4.80;  // 默认通用配置
    tv.vehicle_type = (i < types.size()) ? types[i] : "unknown";
    tv.work_mode = (i < modes.size()) ? modes[i] : "light";
    // ★ 规则：电量 ≤ 阈值 时不能重载 → 自动降级
    bool want_heavy = (tv.work_mode == "heavy" || tv.work_mode == "重载");
    if(want_heavy && bs[i] <= heavy_soc_threshold_ + 1e-9) {
      RCLCPP_WARN(get_logger(),
        "  车#%zu 剩余 %.0f%% ≤ %.0f%%，无法重载 → 自动降级为 %s",
        i, bs[i], heavy_soc_threshold_, downgrade_mode_.c_str());
      tv.work_mode = downgrade_mode_;
      want_heavy = false;
    }
    tv.is_heavy = want_heavy;
    tv.alpha_hi = lookupAlpha(tv.vehicle_type, tv.work_mode);
    tv.alpha_lo = tv.is_heavy ? lookupAlpha(tv.vehicle_type, downgrade_mode_) : tv.alpha_hi;
    tv.rho_hi = (tv.alpha_hi > 1e-6) ? 1.0/tv.alpha_hi : 0.0;
    tv.rho_lo = (tv.alpha_lo > 1e-6) ? 1.0/tv.alpha_lo : 0.0;
    // ★ 截止期 = 电量降到【停机阈值】的时刻（车辆丧失作业能力即视为失败）
    //   重载车：重载段(b⁰→重载下限) + 降级段(重载下限→停机阈值)
    //   其余车：单段(b⁰→停机阈值)
    if(bs[i] <= park_soc_threshold_) {
      tv.deadline = 0.0;                                    // 已停机，立即失效
    } else if(tv.is_heavy) {
      tv.deadline = (bs[i] - heavy_soc_threshold_) * tv.alpha_hi
                  + (heavy_soc_threshold_ - park_soc_threshold_) * tv.alpha_lo;
    } else {
      tv.deadline = (bs[i] - park_soc_threshold_) * tv.alpha_hi;
    }
    tv.original_index = (int)i;
    target_vehicles_.push_back(tv);
  }

  RCLCPP_INFO(get_logger(),
    "[Scheduler] %zu辆车 | 电池%.1fkWh(20%%=%.1f 80%%=%.1f) | 充电%.0fkW效率%.0f%% | 行驶%.1fkWh/km | 桩%.0fkW",
    target_vehicles_.size(), battery_capacity_kwh_,
    battery_capacity_kwh_*soc_low_threshold_, battery_capacity_kwh_*soc_depart_threshold_,
    charge_power_kw_, charge_efficiency_*100, drive_consume_kwh_per_km_, station_charge_power_kw_);
  RCLCPP_INFO(get_logger(), "[Scheduler] 站端周转: %s | 单趟可用 %.0f kWh",
    battery_swap_ ? (std::string("换电 ")+std::to_string((int)swap_time_min_)+" min(换后满电)").c_str()
                  : "桩充至80%",
    battery_swap_ ? battery_capacity_kwh_*(1.0-soc_low_threshold_)
                  : battery_capacity_kwh_*(soc_depart_threshold_-soc_low_threshold_));

  // ★ 逐辆打印: 类型 位置 容量 剩余电量 充电需求
  for(size_t i=0;i<target_vehicles_.size();i++) {
    const auto & tv = target_vehicles_[i];
    double need_kwh = chargeEnergyNeed((int)i);
    double chg_min = chargeTimeMin((int)i);
    RCLCPP_INFO(get_logger(),
      "  车#%zu [%s|%s%s] 位置(%.1f,%.1f) 容量%.0fkWh 剩余%.0f%% | α=%.0f→%.0fmin/%% τ=%.0fmin → 需%.1fkWh(充%.1fmin)",
      i, tv.vehicle_type.c_str(), tv.work_mode.c_str(), tv.is_heavy?"→降级":"",
      tv.pos.x, tv.pos.y, tv.capacity_kwh, tv.remaining_battery,
      tv.alpha_hi, tv.alpha_lo, tv.deadline, need_kwh, chg_min);
  }

  map_sub_ = create_subscription<HADMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ChargeScheduler::onMap, this, std::placeholders::_1));
  trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/charge_scheduler/trigger", 1,
    std::bind(&ChargeScheduler::onTrigger, this, std::placeholders::_1));
  const int online_k = std::max(1, num_chargers_);
  served_subs_by_charger_.reserve(online_k);
  arrived_target_subs_by_charger_.reserve(online_k);
  arrived_station_subs_by_charger_.reserve(online_k);
  pub_sequences_by_charger_.reserve(online_k);
  pub_return_goals_by_charger_.reserve(online_k);
  pub_task_completes_by_charger_.reserve(online_k);
  for(int k=0;k<online_k;k++) {
    const std::string suffix="_"+std::to_string(k+1);
    served_subs_by_charger_.push_back(create_subscription<std_msgs::msg::Bool>(
      "/task_planner/vehicle_served"+suffix,1,
      [this,k](const std_msgs::msg::Bool::ConstSharedPtr msg){
        onVehicleServedForCharger(k,msg);
      }));
    arrived_target_subs_by_charger_.push_back(create_subscription<std_msgs::msg::Bool>(
      "/task_planner/arrived_target"+suffix,1,
      [this,k](const std_msgs::msg::Bool::ConstSharedPtr msg){
        onArrivedTargetForCharger(k,msg);
      }));
    arrived_station_subs_by_charger_.push_back(create_subscription<std_msgs::msg::Bool>(
      "/task_planner/arrived_station"+suffix,1,
      [this,k](const std_msgs::msg::Bool::ConstSharedPtr msg){
        onArrivedStationForCharger(k,msg);
      }));
    pub_sequences_by_charger_.push_back(create_publisher<PredictedObjects>(
      "/task_planner/target_vehicle_sequence"+suffix,rclcpp::QoS(1).transient_local()));
    pub_return_goals_by_charger_.push_back(create_publisher<geometry_msgs::msg::PoseStamped>(
      "/task_planner/return_goal"+suffix,1));
    pub_task_completes_by_charger_.push_back(create_publisher<std_msgs::msg::Bool>(
      "/task_planner/task_complete"+suffix,rclcpp::QoS(1).transient_local()));
  }
  // 保留原单车成员作为充电车1的别名，K=1的全部执行逻辑不变。
  served_sub_=served_subs_by_charger_.front();
  arrived_target_sub_=arrived_target_subs_by_charger_.front();
  arrived_station_sub_=arrived_station_subs_by_charger_.front();
  execution_event_sub_ = create_subscription<std_msgs::msg::String>(
    execution_feedback_topic_, 20,
    std::bind(&ChargeScheduler::onExecutionEvent, this, std::placeholders::_1));

  pub_sequence_=pub_sequences_by_charger_.front();
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/charge_scheduler/target_markers", rclcpp::QoS(1).transient_local());
  pub_return_goal_=pub_return_goals_by_charger_.front();
  pub_task_complete_=pub_task_completes_by_charger_.front();
  assignment_meta_pub_ = create_publisher<std_msgs::msg::String>(
    assignment_meta_topic_, rclcpp::QoS(20).reliable());

  timer_ = create_wall_timer(std::chrono::milliseconds(500),
    std::bind(&ChargeScheduler::onTimer, this));

  RCLCPP_INFO(get_logger(),
    "[Scheduler] 闭环接口: threshold=%.1fmin | event_sim=%s | policy=%s | structured_feedback=%s (%s)",
    reschedule_threshold_, execution_simulation_mode_?"on":"off",
    execution_reschedule_policy_.c_str(),
    prefer_structured_feedback_?"preferred":"optional", execution_feedback_topic_.c_str());
  RCLCPP_INFO(get_logger(), "[Scheduler] 已启动, 等待地图...");
  for(int k=0;k<online_k;k++) {
    RCLCPP_INFO(get_logger(),
      "[Scheduler] 充电车%d接口: sequence/served/arrived_target/arrived_station/return_goal/task_complete 后缀=_%d",
      k+1,k+1);
  }
}

// ============================================================================
//  地图
// ============================================================================
void ChargeScheduler::onMap(const HADMapBin::ConstSharedPtr msg) {
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);
  // 与下层 task_planner 保持同一口径：直接使用完整 laneletLayer。
  // 不在上层额外筛选 roadLanelets，避免上下层节点集合不一致。
  road_lanelets_ = lanelet::utils::query::laneletLayer(lanelet_map_);

  // ★★★ 与下层 task_planner 同源的路网：RoutingGraph + 虚线掉头增广图 ★★★
  buildRoutingGraph();
  extractDashedSegments();
  buildAugmentedGraph();
  precomputeDistanceMatrix();   // 论文 §3.6：离线预计算距离矩阵 D

  map_loaded_ = true;
  RCLCPP_INFO(get_logger(), "[Scheduler] 地图就绪, %zu 条 lanelet（与下层一致）", road_lanelets_.size());
}

// ============================================================================
//  几何工具
// ============================================================================
double ChargeScheduler::euclid(const Point & a, const Point & b) {
  return std::hypot(a.x-b.x, a.y-b.y);
}



// ★ 分段放电：重载仅在电量 > 阈值时成立，跌破后按降级工况继续放电
double ChargeScheduler::socAt(int idx, double t_min) const {
  const auto & tv = target_vehicles_[idx];
  const double t = std::max(0.0, t_min);
  const double P = park_soc_threshold_;      // 停机阈值：到此即停机等待，不再放电
  if(tv.remaining_battery <= P) return tv.remaining_battery;   // 一开始就已停机

  if(!tv.is_heavy) {
    double b = tv.remaining_battery - tv.rho_hi * t;
    return std::max(P, b);                   // ★ 降到阈值即停机
  }
  // 重载段 → 降级段 → 停机
  double t1 = (tv.remaining_battery - heavy_soc_threshold_) * tv.alpha_hi;
  if(t <= t1) return tv.remaining_battery - tv.rho_hi * t;
  double b = heavy_soc_threshold_ - tv.rho_lo * (t - t1);
  return std::max(P, b);                     // ★ 降到阈值即停机
}

// ★ (车型, 工况) → 1% 电量耗时 α_i (min/%)，取产品手册区间的【保守端】(下限=最紧迫)
//   ρ_i = 1/α_i (%/min)，τ_i = α_i · b_i⁰
double ChargeScheduler::lookupAlpha(const std::string & type, const std::string & mode) const {
  struct Row { const char* type; double heavy, light, idle; };
  static const Row TB[] = {
    // 车型            重载  轻载  怠速   (min/%)
    { "电动装载机282",  4.0,  7.0,  60.0 },
    { "电动装载机350",  5.0,  8.0,  90.0 },
    { "电动挖掘机",     3.0, 10.0,  60.0 },
    { "重型叉车",       4.0,  8.0,  60.0 },
  };
  for(const auto & r : TB) {
    if(type == r.type) {
      if(mode == "heavy" || mode == "重载") return r.heavy * alpha_scale_;
      if(mode == "idle"  || mode == "怠速") return r.idle  * alpha_scale_;
      return r.light * alpha_scale_;                                       // 默认轻载
    }
  }
  return alpha_;                            // 未知车型 → 回退全局 alpha
}

// ============================================================================
//  ★ 与下层 task_planner 同源的路网距离
//    RoutingGraph 前进边 + 虚线掉头边增广图 + Dijkstra
// ============================================================================
void ChargeScheduler::buildRoutingGraph() {
  if(!lanelet_map_) return;
  traffic_rules_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  routing_graph_ptr_ = lanelet::routing::RoutingGraph::build(*lanelet_map_, *traffic_rules_ptr_);
  RCLCPP_INFO(get_logger(), "[Scheduler] RoutingGraph 就绪");
}

double ChargeScheduler::laneletLength(const lanelet::ConstLanelet & ll) {
  const auto cl = lanelet::utils::generateFineCenterline(ll, 1.0);
  double len = 0.0;
  for(size_t i=0; i+1<cl.size(); i++)
    len += std::hypot(cl[i+1].x()-cl[i].x(), cl[i+1].y()-cl[i].y());
  return len;
}

double ChargeScheduler::centerlineOffset(const lanelet::ConstLanelet & ll, const Point & pt) {
  const auto cl = lanelet::utils::generateFineCenterline(ll, 1.0);
  double bd = 1e18;
  for(const auto & p : cl) bd = std::min(bd, std::hypot(p.x()-pt.x, p.y()-pt.y));
  return (bd>1e17) ? 0.0 : bd;
}

// ★ 沿中心线：从 pt 的投影点到 lanelet 末端的弧长
double ChargeScheduler::arcToEnd(const lanelet::ConstLanelet & ll, const Point & pt) {
  const auto cl = lanelet::utils::generateFineCenterline(ll, 1.0);
  if(cl.size() < 2) return 0.0;
  size_t k = 0; double bd = 1e18;
  for(size_t i=0;i<cl.size();i++) {
    double d = std::hypot(cl[i].x()-pt.x, cl[i].y()-pt.y);
    if(d < bd) { bd = d; k = i; }
  }
  double len = 0.0;
  for(size_t i=k; i+1<cl.size(); i++)
    len += std::hypot(cl[i+1].x()-cl[i].x(), cl[i+1].y()-cl[i].y());
  return len;
}

// ★ 沿中心线：从 lanelet 起点到 pt 的投影点的弧长
double ChargeScheduler::arcFromStart(const lanelet::ConstLanelet & ll, const Point & pt) {
  const auto cl = lanelet::utils::generateFineCenterline(ll, 1.0);
  if(cl.size() < 2) return 0.0;
  size_t k = 0; double bd = 1e18;
  for(size_t i=0;i<cl.size();i++) {
    double d = std::hypot(cl[i].x()-pt.x, cl[i].y()-pt.y);
    if(d < bd) { bd = d; k = i; }
  }
  double len = 0.0;
  for(size_t i=0; i<k; i++)
    len += std::hypot(cl[i+1].x()-cl[i].x(), cl[i+1].y()-cl[i].y());
  return len;
}

lanelet::ConstLanelet ChargeScheduler::getClosestLanelet(const Point & pt) const {
  lanelet::ConstLanelet closest;
  geometry_msgs::msg::Pose p;
  p.position.x = pt.x; p.position.y = pt.y; p.position.z = 0.0;
  lanelet::Lanelet closest_mut;
  if(lanelet::utils::query::getClosestLanelet(road_lanelets_, p, &closest_mut))
    closest = closest_mut;
  return closest;
}

int ChargeScheduler::laneletIndexOf(const lanelet::ConstLanelet & ll) const {
  for(size_t i=0;i<aug_lanelets_.size();i++)
    if(aug_lanelets_[i].id() == ll.id()) return (int)i;
  return -1;
}

// 提取虚线段及其两侧车道（与下层同逻辑）
void ChargeScheduler::extractDashedSegments() {
  dashed_segments_.clear();
  if(!lanelet_map_) return;
  for(const auto & ls : lanelet_map_->lineStringLayer) {
    if(ls.attributeOr("subtype", std::string("")) != "dashed") continue;
    if(ls.size() < 2) continue;
    DashedSeg seg; seg.way_id = ls.id();
    seg.mid.x = 0.5*(ls.front().x() + ls.back().x());
    seg.mid.y = 0.5*(ls.front().y() + ls.back().y());
    seg.mid.z = 0.0;
    // ★ 沿整条虚线按步长采样：沿线任意位置都可掉头
    seg.pts.clear();
    for(size_t q=0;q+1<ls.size();q++) {
      double x0=ls[q].x(), y0=ls[q].y(), x1=ls[q+1].x(), y1=ls[q+1].y();
      double L=std::hypot(x1-x0, y1-y0);
      int steps = std::max(1, (int)std::ceil(L / std::max(0.5, dashed_sample_step_)));
      for(int t2=0;t2<steps;t2++) {
        double r=(double)t2/steps;
        Point pp; pp.x=x0+(x1-x0)*r; pp.y=y0+(y1-y0)*r; pp.z=0.0;
        seg.pts.push_back(pp);
      }
    }
    { Point pe; pe.x=ls.back().x(); pe.y=ls.back().y(); pe.z=0.0; seg.pts.push_back(pe); }
    // 与下层 task_planner 一致：找共享这条虚线作为 leftBound 的两个 lanelet（互为对面）
    std::vector<lanelet::ConstLanelet> users;
    for(const auto & ll : road_lanelets_)
      if(ll.leftBound().id() == ls.id()) users.push_back(ll);
    if(users.size() != 2) continue;   // 必须恰好 2 个
    seg.lane_a = users[0];
    seg.lane_b = users[1];
    dashed_segments_.push_back(seg);
  }
  RCLCPP_INFO(get_logger(), "[Scheduler] 提取到 %zu 条虚线捷径", dashed_segments_.size());
}

// 增强图：严格复用下层 lanelet 级 following + 虚线掉头拓扑
void ChargeScheduler::buildAugmentedGraph() {
  aug_lanelets_.clear();
  planner_graph_.clear();
  succ_.clear(); dash_exits_.clear();
  node_pos_.clear(); node_off_.clear(); total_nodes_ = 0;
  if(!routing_graph_ptr_) return;

  // 节点集合与下层 task_planner::buildAugmentedGraph 完全一致。
  for(const auto & ll : road_lanelets_) aug_lanelets_.push_back(ll);
  const int n = (int)aug_lanelets_.size();
  planner_graph_.assign(n, {});

  // 缓存细中心线，供 OD 距离几何化时计算起终点部分弧长。
  aug_cl_.assign(n, {});
  aug_len_.assign(n, 0.0);
  for(int i=0;i<n;i++) {
    const auto cl = lanelet::utils::generateFineCenterline(aug_lanelets_[i], 1.0);
    aug_cl_[i].reserve(cl.size());
    for(const auto & p : cl) {
      Point q; q.x=p.x(); q.y=p.y(); q.z=0.0;
      aug_cl_[i].push_back(q);
    }
    for(size_t k=0;k+1<aug_cl_[i].size();k++)
      aug_len_[i] += euclid(aug_cl_[i][k], aug_cl_[i][k+1]);
  }

  // 前进边：严格复制下层口径，边权为“后继 lanelet 的完整中心线长度”。
  for(int i=0;i<n;i++) {
    for(const auto & nxt : routing_graph_ptr_->following(aug_lanelets_[i])) {
      const int j = laneletIndexOf(nxt);
      if(j < 0) continue;
      planner_graph_[i].push_back({j, aug_len_[j], false, lanelet::InvalId});
    }
  }

  // 虚线掉头边：严格复制下层口径，一条虚线对应双向固定代价边。
  for(const auto & seg : dashed_segments_) {
    const int ia = laneletIndexOf(seg.lane_a);
    const int ib = laneletIndexOf(seg.lane_b);
    if(ia < 0 || ib < 0) continue;
    planner_graph_[ia].push_back({ib, uturn_shift_, true, seg.way_id});
    planner_graph_[ib].push_back({ia, uturn_shift_, true, seg.way_id});
  }

  RCLCPP_INFO(get_logger(),
    "[Scheduler] 下层兼容增强图: %d 节点, %zu 条虚线掉头边",
    n, dashed_segments_.size());
}

// 点在某条 lanelet 上的最近投影：off=横向垂距, arc=沿中心线弧长
void ChargeScheduler::laneNearest(int li, const Point & p, double & off, double & arc) const {
  const auto & cl = aug_cl_[li];
  off = 1e18; arc = 0.0;
  if(cl.size() < 2) return;
  size_t k = 0; double bd = 1e18;
  for(size_t i=0;i<cl.size();i++) {
    double d = std::hypot(cl[i].x-p.x, cl[i].y-p.y);
    if(d < bd) { bd = d; k = i; }
  }
  off = bd;
  double a = 0.0;
  for(size_t i=0;i<k;i++) a += std::hypot(cl[i+1].x-cl[i].x, cl[i+1].y-cl[i].y);
  arc = a;
}

// 将中心线弧长转换为投影点。
Point ChargeScheduler::pointAtArc(int li, double arc) const {
  Point p{};
  if(li < 0 || li >= (int)aug_cl_.size() || aug_cl_[li].empty()) return p;
  const auto & cl = aug_cl_[li];
  if(arc <= 0.0) return cl.front();
  double acc = 0.0;
  for(size_t i=0;i+1<cl.size();i++) {
    const double seg = std::hypot(cl[i+1].x-cl[i].x, cl[i+1].y-cl[i].y);
    if(acc + seg >= arc && seg > 1e-9) {
      const double r = (arc-acc)/seg;
      p.x = cl[i].x + r*(cl[i+1].x-cl[i].x);
      p.y = cl[i].y + r*(cl[i+1].y-cl[i].y);
      p.z = 0.0;
      return p;
    }
    acc += seg;
  }
  return cl.back();
}

// 取投影位置附近的中心线单位切向，用于识别共享边界的对向车道。
std::pair<double,double> ChargeScheduler::laneTangentAtArc(int li, double arc) const {
  if(li < 0 || li >= (int)aug_cl_.size() || aug_cl_[li].size() < 2) return {0.0, 0.0};
  const auto & cl = aug_cl_[li];
  double acc = 0.0;
  size_t k = 0;
  for(size_t i=0;i+1<cl.size();i++) {
    const double seg = std::hypot(cl[i+1].x-cl[i].x, cl[i+1].y-cl[i].y);
    if(acc + seg >= arc) { k=i; break; }
    acc += seg;
    k=i;
  }
  const double dx = cl[k+1].x-cl[k].x;
  const double dy = cl[k+1].y-cl[k].y;
  const double n = std::hypot(dx,dy);
  return n > 1e-9 ? std::make_pair(dx/n,dy/n) : std::make_pair(0.0,0.0);
}

bool ChargeScheduler::shareLaneBoundary(int a, int b) const {
  if(a < 0 || b < 0 || a >= (int)aug_lanelets_.size() ||
     b >= (int)aug_lanelets_.size() || a==b) return false;
  const auto & A = aug_lanelets_[a];
  const auto & B = aug_lanelets_[b];
  const auto al=A.leftBound().id(), ar=A.rightBound().id();
  const auto bl=B.leftBound().id(), br=B.rightBound().id();
  return al==bl || al==br || ar==bl || ar==br;
}

const char * ChargeScheduler::abstractPairMethodName(AbstractPairMethod method) const {
  switch(method) {
    case AbstractPairMethod::SharedBoundaryId: return "shared_boundary_id";
    case AbstractPairMethod::GeometricBoundary: return "geometric_boundary";
    case AbstractPairMethod::NormalDirectionFallback: return "normal_direction_fallback";
    default: return "not_found";
  }
}

// 点到LineString的最近点。这里不依赖边界primitive ID，仅使用实际几何。
Point ChargeScheduler::nearestPointOnLineString(
    const lanelet::ConstLineString3d & line, const Point & p) const {
  Point best = p;
  if(line.empty()) return best;
  if(line.size()==1) {
    best.x=line.front().x(); best.y=line.front().y(); best.z=0.0;
    return best;
  }

  double best_d2=1e36;
  for(size_t i=0;i+1<line.size();i++) {
    const double ax=line[i].x(), ay=line[i].y();
    const double bx=line[i+1].x(), by=line[i+1].y();
    const double vx=bx-ax, vy=by-ay;
    const double den=vx*vx+vy*vy;
    double r=0.0;
    if(den>1e-12) r=((p.x-ax)*vx+(p.y-ay)*vy)/den;
    r=std::max(0.0,std::min(1.0,r));
    const double qx=ax+r*vx, qy=ay+r*vy;
    const double dx=qx-p.x, dy=qy-p.y;
    const double d2=dx*dx+dy*dy;
    if(d2<best_d2) {
      best_d2=d2;
      best.x=qx; best.y=qy; best.z=0.0;
    }
  }
  return best;
}

// 判断两条lanelet是否“几何共边”。用于处理物理上共用车道线、但在OSM中
// 被画成两个不同LineString ID的地图。只在共享ID识别失败后使用。
bool ChargeScheduler::geometricallyShareLaneBoundary(
    int a, double arc_a, int b, double /*arc_b*/, double & mean_gap) const {
  mean_gap=1e18;
  if(a<0 || b<0 || a>=(int)aug_lanelets_.size() ||
     b>=(int)aug_lanelets_.size() || a==b) return false;

  const auto & A=aug_lanelets_[a];
  const auto & B=aug_lanelets_[b];
  const std::array<lanelet::ConstLineString3d,2> bounds_a{
    A.leftBound(),A.rightBound()};
  const std::array<lanelet::ConstLineString3d,2> bounds_b{
    B.leftBound(),B.rightBound()};

  const double span=std::max(0.5,abstract_parallel_span_m_);
  const std::array<double,5> offsets{-span,-0.5*span,0.0,0.5*span,span};
  std::vector<double> sample_arcs;
  sample_arcs.reserve(offsets.size());
  for(double d:offsets) {
    const double aa=std::max(0.0,std::min(aug_len_[a],arc_a+d));
    bool duplicate=false;
    for(double old:sample_arcs) if(std::abs(old-aa)<1e-4) { duplicate=true; break; }
    if(!duplicate) sample_arcs.push_back(aa);
  }

  bool found=false;
  for(const auto & ba:bounds_a) for(const auto & bb:bounds_b) {
    int ok=0;
    double sum=0.0;
    for(double aa:sample_arcs) {
      const Point center=pointAtArc(a,aa);
      const Point qa=nearestPointOnLineString(ba,center);
      const Point qb=nearestPointOnLineString(bb,center);
      const double gap=std::hypot(qa.x-qb.x,qa.y-qb.y);
      if(gap<=abstract_boundary_gap_tol_m_+1e-9) {
        ok++;
        sum+=gap;
      }
    }
    const int n=(int)sample_arcs.size();
    const int fraction_need=(int)std::ceil(0.8*std::max(1,n));
    const int need=std::min(n,std::max(1,std::max(
      abstract_boundary_min_samples_,fraction_need)));
    if(ok>=need) {
      const double avg=ok?sum/ok:1e18;
      if(avg<mean_gap) { mean_gap=avg; found=true; }
    }
  }
  return found;
}

// 评价一个局部法向候选：方向近似相反、主要位于当前投影点法向、
// 横向间距有限，并在投影点前后局部范围内保持反向平行。
bool ChargeScheduler::evaluateOppositeCandidate(
    int base, double base_arc, int candidate,
    AbstractCandidateMetrics & metrics) const {
  metrics=AbstractCandidateMetrics();
  metrics.lane=candidate;
  if(base<0 || candidate<0 || base>=(int)aug_lanelets_.size() ||
     candidate>=(int)aug_lanelets_.size() || base==candidate) return false;

  const Point p0=pointAtArc(base,base_arc);
  const auto t0=laneTangentAtArc(base,base_arc);
  if(std::hypot(t0.first,t0.second)<0.5) return false;

  double off=0.0,arc=0.0;
  laneNearest(candidate,p0,off,arc);
  const Point q=pointAtArc(candidate,arc);
  const auto tj=laneTangentAtArc(candidate,arc);
  if(std::hypot(tj.first,tj.second)<0.5) return false;

  const double dx=q.x-p0.x,dy=q.y-p0.y;
  const double nx=-t0.second,ny=t0.first;
  metrics.arc=arc;
  metrics.projected=q;
  metrics.direction_dot=t0.first*tj.first+t0.second*tj.second;
  metrics.longitudinal_offset=std::abs(dx*t0.first+dy*t0.second);
  metrics.lateral_distance=std::abs(dx*nx+dy*ny);
  metrics.euclidean_distance=std::hypot(dx,dy);
  metrics.normal_alignment=metrics.euclidean_distance>1e-9 ?
    metrics.lateral_distance/metrics.euclidean_distance : 0.0;

  const double pi=std::acos(-1.0);
  const double opposite_cos=std::cos(
    std::max(0.0,std::min(89.0,abstract_opposite_angle_deg_))*pi/180.0);
  const double normal_cos=std::cos(
    std::max(0.0,std::min(89.0,abstract_normal_angle_deg_))*pi/180.0);

  if(metrics.direction_dot>-opposite_cos) return false;
  if(metrics.euclidean_distance>abstract_search_radius_m_+1e-9) return false;
  if(metrics.lateral_distance<abstract_min_lateral_m_-1e-9 ||
     metrics.lateral_distance>abstract_max_lateral_m_+1e-9) return false;
  if(metrics.longitudinal_offset>abstract_max_longitudinal_m_+1e-9) return false;
  if(metrics.normal_alignment+1e-9<normal_cos) return false;

  // 在局部前后采样，排除交叉口处“一点很近但并非平行相邻”的误配。
  const double span=std::max(0.5,abstract_parallel_span_m_);
  const std::array<double,3> offsets{-span,0.0,span};
  std::vector<double> arcs;
  for(double d:offsets) {
    const double aa=std::max(0.0,std::min(aug_len_[base],base_arc+d));
    bool duplicate=false;
    for(double old:arcs) if(std::abs(old-aa)<1e-4) { duplicate=true; break; }
    if(!duplicate) arcs.push_back(aa);
  }

  const double relaxed_opp_cos=std::cos(
    std::max(0.0,std::min(89.0,abstract_opposite_angle_deg_+10.0))*pi/180.0);
  const double relaxed_normal_cos=std::cos(
    std::max(0.0,std::min(89.0,abstract_normal_angle_deg_+10.0))*pi/180.0);

  int passed=0;
  for(double aa:arcs) {
    const Point pb=pointAtArc(base,aa);
    const auto tb=laneTangentAtArc(base,aa);
    double o2=0.0,a2=0.0;
    laneNearest(candidate,pb,o2,a2);
    const Point qc=pointAtArc(candidate,a2);
    const auto tc=laneTangentAtArc(candidate,a2);
    const double ddot=tb.first*tc.first+tb.second*tc.second;
    const double vx=qc.x-pb.x,vy=qc.y-pb.y;
    const double nnx=-tb.second,nny=tb.first;
    const double dlong=std::abs(vx*tb.first+vy*tb.second);
    const double dlat=std::abs(vx*nnx+vy*nny);
    const double dist=std::hypot(vx,vy);
    const double align=dist>1e-9?dlat/dist:0.0;
    if(ddot<=-relaxed_opp_cos &&
       dist<=1.25*abstract_search_radius_m_+1e-9 &&
       dlat>=0.5*abstract_min_lateral_m_-1e-9 &&
       dlat<=1.25*abstract_max_lateral_m_+1e-9 &&
       dlong<=1.5*abstract_max_longitudinal_m_+1e-9 &&
       align+1e-9>=relaxed_normal_cos) {
      passed++;
    }
  }
  metrics.parallel_samples=passed;
  return passed==(int)arcs.size() && passed>0;
}

// 每个任务点映射为：最近车道投影 + 相邻反向车道投影。
// 优先保持旧地图的共享边界ID口径；若地图将同一物理边界拆成不同ID，
// 再使用几何共边；最后才使用受约束的局部法向反向搜索。
std::vector<ChargeScheduler::AbstractProjection>
ChargeScheduler::abstractProjections(const Point & p, AbstractPairInfo * info) const {
  std::vector<AbstractProjection> out;
  if(info) *info=AbstractPairInfo{};
  if(aug_lanelets_.empty()) return out;

  const auto nearest=getClosestLanelet(p);
  const int base=laneletIndexOf(nearest);
  if(base<0) return out;

  double off0=0.0,arc0=0.0;
  laneNearest(base,p,off0,arc0);
  const Point p0=pointAtArc(base,arc0);
  out.push_back({base,arc0,p0});
  if(info) info->base_lane=base;

  const auto t0=laneTangentAtArc(base,arc0);
  int best=-1;
  double best_arc=0.0;
  AbstractPairMethod method=AbstractPairMethod::None;
  AbstractCandidateMetrics best_metrics;
  double best_boundary_gap=-1.0;

  // 1) 原始规则：共享同一个边界primitive ID。保留优先级以保证旧地图口径不变。
  double best_off=1e18,best_dot=1.0;
  for(int j=0;j<(int)aug_lanelets_.size();j++) {
    if(!shareLaneBoundary(base,j)) continue;
    double off=0.0,arc=0.0;
    laneNearest(j,p,off,arc);
    const auto tj=laneTangentAtArc(j,arc);
    const double dot=t0.first*tj.first+t0.second*tj.second;
    if(dot>-0.20) continue;
    if(off<best_off-1e-6 || (std::abs(off-best_off)<1e-6 && dot<best_dot)) {
      best=j; best_off=off; best_dot=dot; best_arc=arc;
    }
  }
  if(best>=0) {
    method=AbstractPairMethod::SharedBoundaryId;
    evaluateOppositeCandidate(base,arc0,best,best_metrics); // 仅用于诊断，不改变旧选择
    best_metrics.lane=best;
    best_metrics.arc=best_arc;
    best_metrics.projected=pointAtArc(best,best_arc);
  }

  // 2) 边界ID不同，但局部几何上实际共线/重合。
  if(best<0) {
    double chosen_gap=1e18,chosen_dist=1e18,chosen_dot=1.0;
    for(int j=0;j<(int)aug_lanelets_.size();j++) {
      AbstractCandidateMetrics metrics;
      if(!evaluateOppositeCandidate(base,arc0,j,metrics)) continue;
      double gap=1e18;
      if(!geometricallyShareLaneBoundary(base,arc0,j,metrics.arc,gap)) continue;
      if(gap<chosen_gap-1e-6 ||
         (std::abs(gap-chosen_gap)<1e-6 && metrics.euclidean_distance<chosen_dist-1e-6) ||
         (std::abs(gap-chosen_gap)<1e-6 &&
          std::abs(metrics.euclidean_distance-chosen_dist)<1e-6 &&
          metrics.direction_dot<chosen_dot)) {
        best=j; best_arc=metrics.arc; best_metrics=metrics;
        chosen_gap=gap; chosen_dist=metrics.euclidean_distance;
        chosen_dot=metrics.direction_dot;
      }
    }
    if(best>=0) {
      method=AbstractPairMethod::GeometricBoundary;
      best_boundary_gap=chosen_gap;
    }
  }

  // 3) 受约束的局部法向回退：优先最小纵向偏移，再选最近且方向更相反的车道。
  if(best<0) {
    double chosen_long=1e18,chosen_dist=1e18,chosen_dot=1.0;
    for(int j=0;j<(int)aug_lanelets_.size();j++) {
      AbstractCandidateMetrics metrics;
      if(!evaluateOppositeCandidate(base,arc0,j,metrics)) continue;
      if(metrics.longitudinal_offset<chosen_long-1e-6 ||
         (std::abs(metrics.longitudinal_offset-chosen_long)<1e-6 &&
          metrics.euclidean_distance<chosen_dist-1e-6) ||
         (std::abs(metrics.longitudinal_offset-chosen_long)<1e-6 &&
          std::abs(metrics.euclidean_distance-chosen_dist)<1e-6 &&
          metrics.direction_dot<chosen_dot)) {
        best=j; best_arc=metrics.arc; best_metrics=metrics;
        chosen_long=metrics.longitudinal_offset;
        chosen_dist=metrics.euclidean_distance;
        chosen_dot=metrics.direction_dot;
      }
    }
    if(best>=0) method=AbstractPairMethod::NormalDirectionFallback;
  }

  if(best>=0) {
    out.push_back({best,best_arc,pointAtArc(best,best_arc)});
    if(info) {
      info->method=method;
      info->opposite_lane=best;
      info->lateral_distance=best_metrics.lateral_distance;
      info->longitudinal_offset=best_metrics.longitudinal_offset;
      info->euclidean_distance=best_metrics.euclidean_distance;
      info->direction_dot=best_metrics.direction_dot;
      info->normal_alignment=best_metrics.normal_alignment;
      info->boundary_gap=best_boundary_gap;
      info->parallel_samples=best_metrics.parallel_samples;
    }
  }
  return out;
}

// 普通 following 图上的位置感知距离；不包含任何虚线、掉头或横向机动边。
double ChargeScheduler::abstractDirectedDistance(
    const AbstractProjection & from, const AbstractProjection & to) const {
  const int s=from.lane, t=to.lane;
  if(s<0 || t<0 || !routing_graph_ptr_) return 1e18;

  double best=1e18;
  if(s==t && to.arc>=from.arc-1e-9) best=std::max(0.0,to.arc-from.arc);

  const int n=(int)aug_lanelets_.size();
  std::vector<double> gd(n,1e18);
  using PQ=std::pair<double,int>;
  std::priority_queue<PQ,std::vector<PQ>,std::greater<PQ>> pq;

  // 先走完起点 lanelet 的剩余部分，到达每个普通后继的入口。
  const double leave_cost=std::max(0.0,aug_len_[s]-from.arc);
  for(const auto & nxt:routing_graph_ptr_->following(aug_lanelets_[s])) {
    const int v=laneletIndexOf(nxt);
    if(v<0) continue;
    if(leave_cost<gd[v]) { gd[v]=leave_cost; pq.push({gd[v],v}); }
  }

  while(!pq.empty()) {
    const auto [du,u]=pq.top(); pq.pop();
    if(du>gd[u]+1e-9) continue;
    if(du>=best-1e-9) continue;
    if(u==t) best=std::min(best,du+to.arc);
    for(const auto & nxt:routing_graph_ptr_->following(aug_lanelets_[u])) {
      const int v=laneletIndexOf(nxt);
      if(v<0) continue;
      const double nd=du+aug_len_[u];
      if(nd+1e-9<gd[v]) { gd[v]=nd; pq.push({nd,v}); }
    }
  }
  return best;
}

// 两端各有严格两个投影，计算2×2四条普通路网路径并取最短。
// 投影在矩阵预计算开始时缓存，避免对每个OD重复执行几何搜索。
double ChargeScheduler::abstractDistance(
    const std::vector<AbstractProjection> & A,
    const std::vector<AbstractProjection> & B) const {
  // 投影集合大小由节点附近的实际道路结构决定：
  //   单向驿站可为 1 个投影；普通双向道路目标节点通常为 2 个投影。
  // 在两个可用投影集合的笛卡尔积上取最短普通 following 路径，
  // 因而自动支持 1×1、1×2、2×1 和 2×2 组合。
  if(A.empty() || B.empty()) return 1e18;
  double best=1e18;
  for(const auto & a:A) for(const auto & b:B)
    best=std::min(best,abstractDirectedDistance(a,b));
  return best;
}

// ★ DSP基础距离：仅使用 Lanelet2 RoutingGraph 的有向 following 拓扑。
//   本函数本身不使用虚线掉头/跨线增广边；构造 DSP 矩阵时，仅对 following-only
//   不可达的 OD 使用 ECD 虚线增广结果恢复连通。下层 TaskPlanner 完全不修改。
//   点到路网的接入仍使用最近 lanelet 投影；若目标位于同一 lanelet 的前方可直接到达，
//   若位于后方则必须沿 following 拓扑绕行后重新进入目标 lanelet。
double ChargeScheduler::directedShortestPathDistance(const Point & from, const Point & to) {
  if(planner_graph_.empty() || aug_lanelets_.empty()) return euclid(from, to);

  const lanelet::ConstLanelet lf = getClosestLanelet(from);
  const lanelet::ConstLanelet lt = getClosestLanelet(to);
  const int s = laneletIndexOf(lf), t = laneletIndexOf(lt);
  if(s < 0 || t < 0) return euclid(from, to);

  double off_from=0.0, a_from=0.0, off_to=0.0, a_to=0.0;
  laneNearest(s, from, off_from, a_from);
  laneNearest(t, to,   off_to,   a_to);

  constexpr double INF = 1e18;
  double best = INF;

  // 同一有向 lanelet 且目标在前方：无需离开当前 lanelet。
  if(s == t && a_to >= a_from - 1e-6) {
    best = off_from + std::max(0.0, a_to - a_from) + off_to;
  }

  // gd[u]：从 from 出发并沿有向道路行驶至 lanelet u 末端的最短距离。
  // planner_graph_ 同时含 following 边和虚线掉头边；DSP显式过滤 is_uturn=true 的边。
  const int n = static_cast<int>(planner_graph_.size());
  std::vector<double> gd(n, INF);
  using PQ = std::pair<double,int>;
  std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;

  gd[s] = off_from + std::max(0.0, aug_len_[s] - a_from);
  pq.push({gd[s], s});

  while(!pq.empty()) {
    const auto [du, u] = pq.top();
    pq.pop();
    if(du > gd[u] + 1e-9) continue;
    if(du >= best) continue;

    for(const auto & e : planner_graph_[u]) {
      if(e.is_uturn) continue;  // ★ DSP核心：关闭虚线掉头/跨线捷径
      const int v = e.to;
      if(v < 0 || v >= n) continue;

      // 从 u 末端进入目标 lanelet t 后，只走到目标投影位置即可。
      if(v == t) {
        best = std::min(best, du + std::max(0.0, a_to) + off_to);
      }

      // 若继续穿过 v，则走完整条 v 中心线。
      const double nd = du + aug_len_[v];
      if(nd + 1e-9 < gd[v]) {
        gd[v] = nd;
        pq.push({nd, v});
      }
    }
  }

  if(best >= 1e17) return INF;
  return std::max(euclid(from, to), best);
}

// ★ 下层兼容机动距离：先按 task_planner 的 lanelet 级增强图选路径，
//   再补入 OD 点的部分中心线弧长和局部 A* 停靠距离。
double ChargeScheduler::augmentedDistance(const Point & from, const Point & to) {
  if(planner_graph_.empty() || aug_lanelets_.empty()) return euclid(from, to);

  const lanelet::ConstLanelet lf = getClosestLanelet(from);
  const lanelet::ConstLanelet lt = getClosestLanelet(to);
  const int s = laneletIndexOf(lf), t = laneletIndexOf(lt);
  if(s < 0 || t < 0) return euclid(from, to);

  double off_from=0.0, a_from=0.0, off_to=0.0, a_to=0.0;
  laneNearest(s, from, off_from, a_from);
  laneNearest(t, to,   off_to,   a_to);

  // 先完全按下层 augmentedSearch 的 lanelet 级图和边权选路径。
  const int n = (int)planner_graph_.size();
  std::vector<double> gd(n, 1e18);
  std::vector<int> prev(n, -1);
  std::vector<bool> prev_uturn(n, false), visited(n, false);
  using PQ = std::pair<double,int>;
  std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;
  gd[s] = 0.0; pq.push({0.0, s});
  while(!pq.empty()) {
    const int u = pq.top().second; pq.pop();
    if(visited[u]) continue;
    visited[u] = true;
    if(u == t) break;
    for(const auto & e : planner_graph_[u]) {
      if(gd[u] + e.cost < gd[e.to]) {
        gd[e.to] = gd[u] + e.cost;
        prev[e.to] = u;
        prev_uturn[e.to] = e.is_uturn;
        pq.push({gd[e.to], e.to});
      }
    }
  }
  if(gd[t] >= 1e17) return 1e18;

  struct Step { int lane; bool by_uturn; };
  std::vector<Step> rev;
  for(int cur=t; cur!=-1; cur=prev[cur]) rev.push_back({cur, prev_uturn[cur]});
  std::reverse(rev.begin(), rev.end());

  auto pointAtArc = [&](int li, double arc)->Point {
    Point p{};
    const auto & cl = aug_cl_[li];
    if(cl.empty()) return p;
    if(arc <= 0.0) return cl.front();
    double acc = 0.0;
    for(size_t i=0;i+1<cl.size();i++) {
      const double seg = euclid(cl[i], cl[i+1]);
      if(acc + seg >= arc && seg > 1e-9) {
        const double r = (arc-acc)/seg;
        p.x = cl[i].x + r*(cl[i+1].x-cl[i].x);
        p.y = cl[i].y + r*(cl[i+1].y-cl[i].y);
        p.z = 0.0;
        return p;
      }
      acc += seg;
    }
    return cl.back();
  };

  // 路径选择与下层一致；距离评价补入起终点部分弧长，避免同 lanelet 距离为 0。
  double length = off_from;
  int cur_lane = s;
  double cur_arc = a_from;
  for(size_t i=1;i<rev.size();i++) {
    const int nxt = rev[i].lane;
    if(rev[i].by_uturn) {
      // 复现下层 augmentedPathToTasks 的掉头点规则：
      // 起点掉头=当前位置；最后掉头=目标在源车道的投影；中间掉头≈中心线第2个点。
      Point anchor;
      if(i == 1) anchor = from;
      else if(i + 1 == rev.size()) anchor = to;
      else if(aug_cl_[cur_lane].size() >= 2) anchor = aug_cl_[cur_lane][1];
      else anchor = pointAtArc(cur_lane, cur_arc);

      double dep_off=0.0, dep_arc=0.0;
      laneNearest(cur_lane, anchor, dep_off, dep_arc);
      if(i != 1) {
        if(dep_arc >= cur_arc - 1e-6) length += dep_arc - cur_arc;
        else length += euclid(pointAtArc(cur_lane, cur_arc), pointAtArc(cur_lane, dep_arc));
      }
      length += uturn_shift_;

      double land_off=0.0, land_arc=0.0;
      laneNearest(nxt, anchor, land_off, land_arc);
      cur_lane = nxt;
      cur_arc = land_arc;
    } else {
      length += std::max(0.0, aug_len_[cur_lane] - cur_arc);
      cur_lane = nxt;
      cur_arc = 0.0;
    }
  }

  if(cur_lane == t && a_to >= cur_arc - 1e-6) {
    length += (a_to - cur_arc) + off_to;
  } else {
    // 与下层最终 A* 停靠段相对应；目标位于当前投影后方时以局部直连估计。
    length += euclid(pointAtArc(cur_lane, cur_arc), to);
  }

  // 调度距离不能短于几何直线，防止固定动作代价造成非物理小于欧氏距离。
  return std::max(euclid(from, to), length);
}

// ============================================================================
//  ★ 论文 §3.6：离线预计算距离矩阵 D（0=驿站, 1..n=目标车）
// ============================================================================
void ChargeScheduler::precomputeDistanceMatrix() {
  const int n=(int)target_vehicles_.size();
  const int m=n+1;
  dist_matrix_.assign(m,std::vector<double>(m,0.0));
  dist_matrix_real_.assign(m,std::vector<double>(m,0.0));
  dist_matrix_dsp_.assign(m,std::vector<double>(m,0.0));
  dist_matrix_abstract_.assign(m,std::vector<double>(m,0.0));

  std::vector<Point> nodes;
  nodes.push_back(charging_station_);
  for(const auto & tv:target_vehicles_) nodes.push_back(tv.pos);

  if(distance_mode_!="maneuver" && distance_mode_!="dsp" &&
     distance_mode_!="euclid" && distance_mode_!="abstract") {
    RCLCPP_WARN(get_logger(),"[Scheduler] 未知 distance_mode=%s，回退 maneuver(ECD)",distance_mode_.c_str());
    distance_mode_="maneuver";
  }

  auto t0=std::chrono::steady_clock::now();
  const bool need_dsp=(distance_mode_=="dsp");
  const bool need_abstract=(distance_mode_=="abstract");
  int unreachable_real=0, unreachable_dsp_raw=0, dsp_dashed_fallback=0, unreachable_dsp_final=0, unreachable_abs=0;
  int missing_pair_nodes=0;
  int pair_by_id=0,pair_by_geometry=0,pair_by_normal=0;
  int station_single_lane=0;
  std::vector<std::vector<AbstractProjection>> abstract_node_projections(m);
  std::vector<AbstractPairInfo> abstract_pair_info(m);
  if(need_abstract) {
    RCLCPP_INFO(get_logger(),
      "  [Abstract参数] opposite_angle=%.1fdeg normal_angle=%.1fdeg search_radius=%.2fm lateral=[%.2f,%.2f]m longitudinal<=%.2fm parallel_span=%.2fm boundary_gap<=%.3fm",
      abstract_opposite_angle_deg_,abstract_normal_angle_deg_,
      abstract_search_radius_m_,abstract_min_lateral_m_,
      abstract_max_lateral_m_,abstract_max_longitudinal_m_,
      abstract_parallel_span_m_,abstract_boundary_gap_tol_m_);
    for(int i=0;i<m;i++) {
      auto & info=abstract_pair_info[i];
      auto & ap=abstract_node_projections[i];
      ap=abstractProjections(nodes[i],&info);
      const std::string name=i==0?"站":("车#"+std::to_string(i-1));
      if(i==0 && ap.size()==1) {
        // 第二张地图的驿站物理上仅连接一条单向车道。
        // 保留最近车道主投影，不虚构不存在的对向车道。
        station_single_lane++;
        RCLCPP_INFO(get_logger(),
          "  [Abstract] 站 → lanelet(%lld) method=station_single_lane | 单向驿站保留1个主投影",
          (long long)aug_lanelets_[ap[0].lane].id());
      } else if(ap.size()!=2) {
        // 目标节点仍要求构造相邻反向车道投影；其失败不能静默降级。
        missing_pair_nodes++;
        RCLCPP_ERROR(get_logger(),
          "  [Abstract] %s 仅找到 %zu 个投影（共享边界ID、几何共边和法向反向搜索均失败），相关OD将记为不可达",
          name.c_str(),ap.size());
      } else {
        if(info.method==AbstractPairMethod::SharedBoundaryId) pair_by_id++;
        else if(info.method==AbstractPairMethod::GeometricBoundary) pair_by_geometry++;
        else if(info.method==AbstractPairMethod::NormalDirectionFallback) pair_by_normal++;
        RCLCPP_INFO(get_logger(),
          "  [Abstract] %s → lanelet(%lld, %lld) method=%s | lateral=%.3fm longitudinal=%.3fm distance=%.3fm dot=%.4f normal=%.4f boundary_gap=%.4fm parallel_samples=%d",
          name.c_str(),
          (long long)aug_lanelets_[ap[0].lane].id(),
          (long long)aug_lanelets_[ap[1].lane].id(),
          abstractPairMethodName(info.method),
          info.lateral_distance,info.longitudinal_offset,
          info.euclidean_distance,info.direction_dot,
          info.normal_alignment,info.boundary_gap,info.parallel_samples);
      }
    }
    RCLCPP_INFO(get_logger(),
      "  [Abstract投影汇总] shared_boundary_id=%d geometric_boundary=%d normal_direction_fallback=%d station_single_lane=%d not_found=%d",
      pair_by_id,pair_by_geometry,pair_by_normal,station_single_lane,missing_pair_nodes);
  }

  for(int i=0;i<m;i++) for(int j=0;j<m;j++) {
    if(i==j) continue;
    // ECD矩阵始终保留，用于所有 surrogate 模式的统一执行口径重放。
    dist_matrix_real_[i][j]=augmentedDistance(nodes[i],nodes[j]);
    if(dist_matrix_real_[i][j]>1e17) unreachable_real++;

    if(distance_mode_=="euclid") {
      dist_matrix_[i][j]=euclid(nodes[i],nodes[j]);
    } else if(need_dsp) {
      // DSP先只使用有向 following 拓扑。仅当该 OD 在 following-only 图上不可达时，
      // 才用 ECD 增广图（含指定虚线机动）恢复可达性；若 following-only 已可达，
      // 绝不使用虚线来缩短其路径。这样 ECD-vs-DSP 只检验“可选虚线捷径”的收益，
      // 而不会因为基础连通性缺失导致 DSP 无可行调度方案。
      dist_matrix_dsp_[i][j]=directedShortestPathDistance(nodes[i],nodes[j]);
      if(dist_matrix_dsp_[i][j]>1e17) {
        unreachable_dsp_raw++;
        if(dist_matrix_real_[i][j]<1e17) {
          dist_matrix_dsp_[i][j]=dist_matrix_real_[i][j];
          dsp_dashed_fallback++;
        }
      }
      if(dist_matrix_dsp_[i][j]>1e17) unreachable_dsp_final++;
      dist_matrix_[i][j]=dist_matrix_dsp_[i][j];
    } else if(need_abstract) {
      dist_matrix_abstract_[i][j]=abstractDistance(
        abstract_node_projections[i],abstract_node_projections[j]);
      if(dist_matrix_abstract_[i][j]>1e17) unreachable_abs++;
      dist_matrix_[i][j]=dist_matrix_abstract_[i][j];
    } else {
      dist_matrix_[i][j]=dist_matrix_real_[i][j];
    }
  }

  const double ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-t0).count();
  matrix_ready_=true;
  RCLCPP_INFO(get_logger(),
    "[Scheduler] 距离矩阵完成 %dx%d, %.1f ms | 模式=%s | ECD不可达=%d DSP原始不可达=%d DSP虚线兜底=%d DSP最终不可达=%d 抽象不可达=%d | 缺少对向投影节点=%d | 对向识别(ID/几何/法向)=%d/%d/%d | 单向驿站投影=%d",
    m,m,ms,distance_mode_.c_str(),unreachable_real,unreachable_dsp_raw,dsp_dashed_fallback,unreachable_dsp_final,unreachable_abs,
    missing_pair_nodes,pair_by_id,pair_by_geometry,pair_by_normal,
    station_single_lane);

  // 真实机动/欧氏比值自检。
  {
    auto nameOf=[&](int k){return k==0?std::string("站"):("车#"+std::to_string(k-1));};
    int bad=0,cnt=0,wi=-1,wj=-1,bi=-1,bj=-1;
    double worst_lo=1e18,sum=0.0,max_r=0.0;
    for(int i=0;i<m;i++) for(int j=0;j<m;j++) {
      if(i==j || dist_matrix_real_[i][j]>1e17) continue;
      const double dl=euclid(nodes[i],nodes[j]);
      if(dl<1e-6) continue;
      const double r=dist_matrix_real_[i][j]/dl;
      sum+=r; cnt++;
      if(r>max_r){max_r=r;bi=i;bj=j;}
      if(r<0.999){bad++;if(r<worst_lo){worst_lo=r;wi=i;wj=j;}}
    }
    if(bad==0) RCLCPP_INFO(get_logger(),
      "  ✓ 比值自检：%d 个 OD 满足 机动/直线≥1 | 均值=%.2f 最大=%.2f (%s→%s)",
      cnt,cnt?sum/cnt:0.0,max_r,bi>=0?nameOf(bi).c_str():"-",bj>=0?nameOf(bj).c_str():"-");
    else RCLCPP_WARN(get_logger(),"  ✗ 比值自检：%d 对<1，最差 %s→%s=%.2f",
      bad,nameOf(wi).c_str(),nameOf(wj).c_str(),worst_lo);
  }

  // DSP矩阵诊断：仅在 DSP 模式下计算。
  // DSP 采用“following-only 优先、不可达才用虚线兜底”。因此：
  // 1) following-only 可达的 OD 应满足 DSP >= ECD；
  // 2) 由虚线兜底恢复的 OD 有 DSP == ECD；
  // 3) 最终不应再因关闭虚线而产生新增不可达 OD（前提是 ECD 本身可达）。
  if(need_dsp) {
    int finite=0,dsp_lt_ecd=0;
    double max_asym=0.0,sum_ratio=0.0,max_ratio=0.0;
    for(int i=0;i<m;i++) for(int j=i+1;j<m;j++) {
      const double a=dist_matrix_dsp_[i][j], ar=dist_matrix_dsp_[j][i];
      if(a<1e17 && ar<1e17) max_asym=std::max(max_asym,std::abs(a-ar));
    }
    for(int i=0;i<m;i++) for(int j=0;j<m;j++) {
      if(i==j || dist_matrix_dsp_[i][j]>1e17 || dist_matrix_real_[i][j]>1e17) continue;
      finite++;
      if(dist_matrix_dsp_[i][j]+1e-6<dist_matrix_real_[i][j]) dsp_lt_ecd++;
      const double r=dist_matrix_dsp_[i][j]/std::max(1e-9,dist_matrix_real_[i][j]);
      sum_ratio+=r; max_ratio=std::max(max_ratio,r);
    }
    RCLCPP_INFO(get_logger(),
      "  [DSP诊断] 有限OD=%d | following-only不可达=%d | 虚线兜底=%d | 最终不可达=%d | DSP<ECD=%d | DSP/ECD均值=%.3f 最大=%.3f | 最大非对称差=%.3fm",
      finite,unreachable_dsp_raw,dsp_dashed_fallback,unreachable_dsp_final,dsp_lt_ecd,
      finite?sum_ratio/finite:0.0,max_ratio,max_asym);
  }

  // 抽象矩阵诊断：它应近似对称，但不强行对称化，避免掩盖地图连接问题。
  if(need_abstract) {
    int finite=0,abs_lt_real=0; double max_asym=0.0,sum_ratio=0.0,max_ratio=0.0;
    for(int i=0;i<m;i++) for(int j=i+1;j<m;j++) {
      const double a=dist_matrix_abstract_[i][j], ar=dist_matrix_abstract_[j][i];
      if(a<1e17 && ar<1e17) max_asym=std::max(max_asym,std::abs(a-ar));
    }
    for(int i=0;i<m;i++) for(int j=0;j<m;j++) {
      if(i==j || dist_matrix_abstract_[i][j]>1e17 || dist_matrix_real_[i][j]>1e17) continue;
      finite++;
      if(dist_matrix_abstract_[i][j]+1e-6<dist_matrix_real_[i][j]) abs_lt_real++;
      const double r=dist_matrix_abstract_[i][j]/std::max(1e-9,dist_matrix_real_[i][j]);
      sum_ratio+=r; max_ratio=std::max(max_ratio,r);
    }
    RCLCPP_INFO(get_logger(),
      "  [Abstract诊断] 有限OD=%d | ABS<MAN=%d | ABS/MAN均值=%.3f 最大=%.3f | 最大非对称差=%.3fm",
      finite,abs_lt_real,finite?sum_ratio/finite:0.0,max_ratio,max_asym);
  }
}

// ============================================================================
//  单趟距离 / 耗电
// ============================================================================
double ChargeScheduler::tripDistance(const Trip & trip) {
  if(trip.empty()) return 0.0;
  double dist = 0.0;
  int cur = 0;                       // 0 = 驿站
  for(int idx : trip) {
    dist += D(cur, idx + 1);         // 查预计算矩阵，O(1)
    cur = idx + 1;
  }
  dist += D(cur, 0);                 // 回站
  return dist;
}


// ============================================================================
//  内层评估器: 正向模拟多趟
//  检查 C2(充电电池) / C4(行驶电池, 每趟距离≤续航) / C6(时间窗)
//  C7 方案C充电决策: z^k=1 ⟺ 残余 < 下一趟需求
// ============================================================================
// ★ 目标车充到80%需要消耗充电车的电量(kWh, 含97%效率)
double ChargeScheduler::chargeEnergyNeed(int idx, double t_min) {
  const auto & tv = target_vehicles_[idx];
  // ★ 持续放电：到达时刻 t 的实际电量 soc_i(t) = max(0, soc_i0 − ρ·t)
  double soc_now = socAt(idx, t_min);                       // ★ 分段放电后的实际电量
  double soc_gap = target_charge_target_ - soc_now/100.0;  // 到80%的缺口(比例)
  if(soc_gap <= 0) return 0.0;
  double energy_to_vehicle = tv.capacity_kwh * soc_gap;   // 打进目标车的电
  return energy_to_vehicle / charge_efficiency_;          // 充电车实际消耗(效率97%)
}

// ★ 充这辆车的时间(min): 打进目标车的电 / 充电功率
double ChargeScheduler::chargeTimeMin(int idx, double t_min) {
  const auto & tv = target_vehicles_[idx];
  double soc_now = socAt(idx, t_min);
  double soc_gap = target_charge_target_ - soc_now/100.0;
  if(soc_gap <= 0) return 0.0;
  double energy_to_vehicle = tv.capacity_kwh * soc_gap;
  return energy_to_vehicle / charge_power_kw_ * 60.0;     // kWh / kW * 60 = min
}

// ★★★ 能量感知动态分趟模拟 ★★★
// 输入: 访问顺序order; 输出: 分好的趟out_trips + 评估结果

// ============================================================================
//  ★ 多车并行解码（K≥2）：K 台充电车各自独立时间线/SoC/趟次/换电
//    列表调度：按给定顺序，把每辆目标车分派给"能最早完成服务"的那台充电车
// ============================================================================
PlanResult ChargeScheduler::simulateOrderParallel(const std::vector<int> & order, Plan & out_trips) {
  const int n = (int)target_vehicles_.size();
  const int K = std::max(2, num_chargers_);
  const double cap = battery_capacity_kwh_;
  const double low = cap * soc_low_threshold_;
  const double gamma = drive_consume_kwh_per_km_;

  PlanResult res;
  res.arrival_time.assign(n, -1.0);
  res.on_time.assign(n, false);
  res.feasible = true;
  res.num_trips = 0;
  out_trips.clear();
  trip_charger_.clear();

  // 各充电车状态
  std::vector<double> t_k(K, 0.0);        // 可用时刻
  std::vector<double> soc_k(K, cap);      // 当前电量(首趟满电)
  std::vector<int>    node_k(K, 0);       // 当前位置(0=驿站)
  std::vector<Trip>   trip_k(K);          // 当前趟

  auto closeTrip = [&](int k){
    if(!trip_k[k].empty()) { out_trips.push_back(trip_k[k]); trip_charger_.push_back(k); res.num_trips++; trip_k[k].clear(); }
  };

  for(int v : order) {
    int best_k = -1; double best_finish = 1e18;
    bool best_swap = false; double best_arr = 0.0, best_soc = 0.0;
    double best_dist_inc = 0.0;

    for(int k=0;k<K;k++) {
      // 方案①：直接从当前位置去服务
      double leg = D(node_k[k], v + 1);
      if(leg < 1e17) {
        double t_arr  = t_k[k] + leg / avg_speed_;
        double e_dr   = leg/1000.0 * gamma;
        double e_ch   = chargeEnergyNeed(v, t_arr);
        double ret    = D(v + 1, 0);
        if(ret >= 1e17) continue;
        double e_ret  = ret/1000.0*gamma;
        if(soc_k[k] - e_dr - e_ch - e_ret >= low - 1e-9) {
          double fin = t_arr + chargeTimeMin(v, t_arr);
          if(fin < best_finish) { best_finish=fin; best_k=k; best_swap=false;
                                  best_arr=t_arr; best_soc=soc_k[k]-e_dr-e_ch;
                                  best_dist_inc=leg; }
        }
      }
      // 方案②：先回站换电(满电)再去服务
      double back = D(node_k[k], 0);
      double leg2 = D(0, v + 1);
      if(back < 1e17 && leg2 < 1e17) {
        const double e_back = back/1000.0*gamma;
        const double soc_at_station = soc_k[k] - e_back;
        if(soc_at_station < low - 1e-9) continue; // 当前电量不足以安全返站
        const double station_wait = battery_swap_ ? swap_time_min_
          : std::max(0.0, cap*soc_depart_threshold_ - soc_at_station)
            / station_charge_power_kw_ * 60.0;
        double t_dep  = t_k[k] + back/avg_speed_ + station_wait;
        double soc0   = battery_swap_ ? cap : cap*soc_depart_threshold_;
        double t_arr  = t_dep + leg2/avg_speed_;
        double e_dr   = leg2/1000.0*gamma;
        double e_ch   = chargeEnergyNeed(v, t_arr);
        double ret    = D(v + 1, 0);
        if(ret >= 1e17) continue;
        double e_ret  = ret/1000.0*gamma;
        if(soc0 - e_dr - e_ch - e_ret >= low - 1e-9) {
          double fin = t_arr + chargeTimeMin(v, t_arr);
          if(fin < best_finish) { best_finish=fin; best_k=k; best_swap=true;
                                  best_arr=t_arr; best_soc=soc0-e_dr-e_ch;
                                  best_dist_inc=back+leg2; }
        }
      }
    }

    if(best_k < 0) {
      res.feasible = false;
      res.energy_violation_count++;
      continue;
    }   // 任何车都服务不了

    int k = best_k;
    if(best_swap) { closeTrip(k); node_k[k] = 0; }        // 回站换电 → 结束上一趟
    res.arrival_time[v] = best_arr;
    bool ok = (best_arr <= target_vehicles_[v].deadline + 1e-6);
    res.on_time[v] = ok;
    if(ok) res.on_time_count++;
    else   res.total_overtime += best_arr - target_vehicles_[v].deadline;
    res.served_count++;

    res.total_distance += best_dist_inc;
    t_k[k]    = best_finish;
    soc_k[k]  = best_soc;
    node_k[k] = v + 1;
    trip_k[k].push_back(v);
  }

  // 收尾：各车回站
  for(int k=0;k<K;k++) {
    if(!trip_k[k].empty()) {
      double back = D(node_k[k], 0);
      if(back < 1e17) {
        t_k[k] += back/avg_speed_;
        res.total_distance += back;
        soc_k[k] -= back/1000.0*gamma;
        if(soc_k[k] < low - 1e-9) {
          res.feasible = false;
          res.energy_violation_count++;
        }
      } else {
        res.feasible = false;
        res.energy_violation_count++;
      }
      closeTrip(k);
    }
  }

  res.total_time = *std::max_element(t_k.begin(), t_k.end());   // makespan = 最晚完工
  return res;
}

PlanResult ChargeScheduler::simulateOrder(const std::vector<int> & order, Plan & out_trips) {
  // ★ K≥2 走并行解码；K=1 保持原有单车逻辑完全不变
  if(num_chargers_ >= 2) return simulateOrderParallel(order, out_trips);

  int n = (int)target_vehicles_.size();
  PlanResult res;
  res.arrival_time.assign(n, -1.0);
  res.on_time.assign(n, false);
  res.feasible = true;
  res.num_trips = 0;
  out_trips.clear();
  trip_charger_.clear();

  double cap = battery_capacity_kwh_;
  double low = cap * soc_low_threshold_;      // 20%阈值
  double depart80 = cap * soc_depart_threshold_; // 80%

  std::vector<bool> served(n, false);
  std::vector<int> pending(order);  // 待服务(按顺序)
  double total_time = rs_start_time_;   // ★ 重调度：从实际时刻续算
  bool first_trip = true;

  // ★★★ 双车交替接力：任一时刻仅一台在外作业，另一台在站补电 ★★★
  //   在外作业时长 T_trip 即为站内那台的充电时长（站端仅一个充电接口）
  const int K = std::max(1, num_chargers_);
  std::vector<double> soc_ch(K, cap);   // 各充电车当前电量(首次均满电)
  int cur_ch = 0;                       // 当前出车的充电车
  double trip_start_time = 0.0;

  while(true) {
    // 还有没服务的车?
    bool any_left=false;
    for(int idx : pending) if(!served[idx]){any_left=true;break;}
    if(!any_left) break;

    // 开始新一趟
    // ★ 重调度：首段使用实际起点/实际 SoC；离线首规划则用 驿站/满电
    double soc = first_trip ? ((rs_start_soc_ >= 0.0) ? rs_start_soc_ : soc_ch[cur_ch])
                            : soc_ch[cur_ch];   // 换电模式下为满电 cap
    trip_start_time = total_time;
    Trip trip;
    int cur_node = first_trip ? rs_start_node_ : 0;   // 0=驿站, 否则=车idx+1
    // ★ 时间用全局累计 total_time(跨趟不清零), 截止期用全局时刻判断

    while(true) {
      // 从pending中按顺序找第一辆未服务的车
      int next = -1;
      for(int idx : pending) if(!served[idx]){ next=idx; break; }
      if(next < 0) break;  // 全服务完

      // 检查：服务后必须仍保留“返回驿站并保持最低 SoC”的能量。
      const double seg_next = D(cur_node, next + 1);
      const double ret_next = D(next + 1, 0);
      const bool reachable_next = (seg_next < 1e17 && ret_next < 1e17);
      const double d_km = reachable_next ? seg_next/1000.0 : 1e18;
      const double t_arr_next = reachable_next ? total_time + seg_next/avg_speed_ : 1e18;
      const double drive_e = d_km * drive_consume_kwh_per_km_;
      const double charge_e = reachable_next ? chargeEnergyNeed(next, t_arr_next) : 1e18;
      const double return_e = reachable_next ? ret_next/1000.0*drive_consume_kwh_per_km_ : 1e18;
      double soc_after = soc - drive_e - charge_e;

      if(reachable_next && soc_after - return_e >= low - 1e-9) {
        // 能充: 充这辆
        double seg_m = seg_next;
        res.total_distance += seg_m;
        total_time += seg_m / avg_speed_;      // 行驶到车(全局累计)
        res.arrival_time[next] = total_time;   // ★ 全局到达时刻(开始充)
        res.served_count++;
        bool ok = (total_time <= target_vehicles_[next].deadline + 1e-6);
        res.on_time[next]=ok;
        if(ok) res.on_time_count++;
        else { res.total_overtime += total_time - target_vehicles_[next].deadline; }  // ★软截止期：超时计入目标，不判不可行
        total_time += chargeTimeMin(next, res.arrival_time[next]);   // ★ 放电感知充电时长
        soc = soc_after;
        cur_node = next + 1;
        served[next]=true;
        trip.push_back(next);
      } else {
        // 不能充next: 从所有未服务车找"充完≥20%"的最优车(加权评分)
        int best=-1; double best_score=-1e18;
        for(int idx : pending) {
          if(served[idx]) continue;
          const double leg_i = D(cur_node, idx + 1);
          const double ret_i = D(idx + 1, 0);
          if(leg_i > 1e17 || ret_i > 1e17) continue;
          double dk = leg_i/1000.0;
          double t_arr_i = total_time + leg_i/avg_speed_;
          double de = dk*drive_consume_kwh_per_km_;
          double ce = chargeEnergyNeed(idx, t_arr_i);
          double re = ret_i/1000.0*drive_consume_kwh_per_km_;
          if(soc - de - ce - re < low - 1e-9) continue;  // 服务后无法安全返站
          // 加权评分: 距离近(权重高) + 电量低 + 需求小
          // ★ 紧迫度：松弛 slack = τ_i − 预计到达时刻；越小越优先
          //   已超时(slack≤0)的车无论服务与否都计入 N_late，故不再抢占容量
          double slack = target_vehicles_[idx].deadline - t_arr_i;
          double urg = (slack > 0.0) ? 1.0/(slack/60.0 + 0.1) : 0.0;
          double sc = w_urgency_ * urg
                    + w_distance_*(1.0/(dk+0.1))
                    + w_battery_*(1.0/(target_vehicles_[idx].remaining_battery+1.0))
                    + w_demand_*(1.0/(ce+0.1));
          if(sc > best_score){best_score=sc; best=idx;}
        }
        if(best >= 0) {
          // 换车: 充best
          double seg_m = D(cur_node, best + 1);
          res.total_distance += seg_m;
          total_time += seg_m/avg_speed_;      // 行驶(全局累计)
          res.arrival_time[best]=total_time;   // ★ 全局到达时刻
          res.served_count++;
          bool ok=(total_time<=target_vehicles_[best].deadline+1e-6);
          res.on_time[best]=ok;
          if(ok)res.on_time_count++;
          else { res.total_overtime += total_time - target_vehicles_[best].deadline; }  // ★软截止期
          total_time += chargeTimeMin(best, res.arrival_time[best]);   // ★ 放电感知
          double dk=seg_m/1000.0;
          soc -= dk*drive_consume_kwh_per_km_ + chargeEnergyNeed(best, res.arrival_time[best]);
          cur_node = best + 1;
          served[best]=true;
          trip.push_back(best);
        } else {
          // 任何车都充不了 → 本趟结束, 返程
          break;
        }
      }
    }

    // 回站
    double back_m = D(cur_node, 0);
    if(back_m > 1e17) {
      res.feasible = false;
      res.energy_violation_count++;
      back_m = 0.0;
    }
    res.total_distance += back_m;
    total_time += back_m/avg_speed_;           // 返程行驶(全局累计)
    soc -= (back_m/1000.0)*drive_consume_kwh_per_km_;
    if(soc < low - 1e-9) {
      res.feasible = false;
      res.energy_violation_count++;
    }

    soc_ch[cur_ch] = soc;                       // 本车回站时的剩余电量

    bool has_more=false;
    for(int idx : pending) if(!served[idx]){has_more=true;break;}
    if(has_more) {
      if(battery_swap_) {
        // ★ 换电模式：固定耗时换电池，换后满电出发（与电量无关）
        total_time += swap_time_min_;
        soc_ch[cur_ch] = cap;
      } else if(K >= 2) {
        // ★ 交替接力：本趟在外时长 = 站内那台的充电时长（并行，无需额外等待）
        double T_trip = total_time - trip_start_time;
        int nxt = (cur_ch + 1) % K;
        soc_ch[nxt] = std::min(depart80, soc_ch[nxt] + station_charge_power_kw_ * T_trip / 60.0);
        // 若接力车电量仍低于最低出发线，则在站补足（此时占用充电口，计入时间）
        double need_min = min_depart_soc_ * cap - soc_ch[nxt];
        if(need_min > 0) {
          double wait = need_min / station_charge_power_kw_ * 60.0;
          total_time += wait;
          soc_ch[nxt] += need_min;
        }
        cur_ch = nxt;                            // 换车出发
      } else {
        // 串行：同一台车回站补到 80% 再出发
        double need = depart80 - soc;
        if(need > 0) total_time += need / station_charge_power_kw_ * 60.0;
        soc_ch[cur_ch] = depart80;
      }
    }

    if(!trip.empty()) {
      out_trips.push_back(trip);
      trip_charger_.push_back(0);
      res.num_trips++;
    }
    first_trip = false;

    // 死循环保护: 如果这趟一辆没充(所有车都充不动) → 不可行, 退出
    if(trip.empty()) { res.feasible=false; break; }
  }

  res.total_time = total_time;
  return res;
}


bool ChargeScheduler::toverFirst() const {
  return objective_order_ == "tover_first";
}

std::string ChargeScheduler::objectiveOrderLabel() const {
  return toverFirst() ? "Tover,Nlate,Ctotal" : "Nlate,Tover,Ctotal";
}

// ============================================================================
//  方案比较: 字典序（可行性/完整服务始终优先）
//  C8降级: 都不可行时比按时车辆数
// ============================================================================
bool ChargeScheduler::isBetter(const PlanResult & a, const PlanResult & b) {
  // 硬约束与服务完整性优先于两个运营目标，避免用软指标奖励不可执行方案。
  if(a.feasible != b.feasible) return a.feasible;
  if(a.served_count != b.served_count) return a.served_count > b.served_count;

  const int M = (int)target_vehicles_.size();
  const int la = M - a.on_time_count;
  const int lb = M - b.on_time_count;
  if(toverFirst()) {
    // OBJ-T: lex-min(T_overtime, N_late, C_total)
    if(std::abs(a.total_overtime - b.total_overtime) > 1e-6)
      return a.total_overtime < b.total_overtime;
    if(la != lb) return la < lb;
  } else {
    // OBJ-N: lex-min(N_late, T_overtime, C_total)
    if(la != lb) return la < lb;
    if(std::abs(a.total_overtime - b.total_overtime) > 1e-6)
      return a.total_overtime < b.total_overtime;
  }
  if(std::abs(a.total_distance - b.total_distance) > 1e-6)
    return a.total_distance < b.total_distance;
  return a.total_time < b.total_time;
}

// ============================================================================
//  精确求解 (n≤8): 枚举所有 分趟+排序 组合
// ============================================================================
void ChargeScheduler::solveExact() {
  int n = (int)target_vehicles_.size();
  std::vector<int> order(n);
  std::iota(order.begin(), order.end(), 0);

  Plan best_plan;
  std::vector<int> best_order;
  PlanResult best_result;
  best_result.feasible = false;
  best_result.num_trips = 999999;
  best_result.total_distance = 1e18;
  best_result.total_time = 1e18;
  best_result.on_time_count = -1;

  // ★ 枚举所有访问顺序(全排列), simulateOrder动态分趟
  std::sort(order.begin(), order.end());
  do {
    Plan trips;
    PlanResult r = simulateOrder(order, trips);
    if(isBetter(r, best_result)) {
      best_result = r;
      best_plan = trips;
      best_order = order;
    }
  } while(std::next_permutation(order.begin(), order.end()));

  planned_order_ = best_order;
  final_result_ = simulateOrder(planned_order_, final_plan_); // 最终重放，固定趟归属映射
  RCLCPP_INFO(get_logger(), "[精确] 最优: 趟数=%d 总时间=%.1fmin 路程=%.1fm 可行=%d 按时=%d/%d",
    final_result_.num_trips, final_result_.total_time, final_result_.total_distance,
    final_result_.feasible, final_result_.on_time_count, n);
}

// (旧)递归枚举趟划分 - 保留但不再使用

// ============================================================================
//  启发式 (n>8): 初始解 + 大邻域搜索
// ============================================================================
// ============================================================================
//  启发式求解：ALNS（主）/ 局部搜索（对照基线）
// ============================================================================

// SA 接受用的标量代价：可行解只看 makespan；不可行解重罚
double ChargeScheduler::planCost(const PlanResult & r) const {
  // 模拟退火、插入与regret算子的标量引导必须随目标顺序同步改变。
  // 最终最好解仍由 isBetter 的严格字典序判定，因此这里不是论文中的加权目标。
  const int M = (int)target_vehicles_.size();
  const double N_late = (double)(M - r.on_time_count);
  double c = 0.0;
  if(toverFirst()) {
    // OBJ-T：让累计超时主导搜索；其次迟到车辆数，最后路程。
    // 采用强层级系数，避免修复/退火仍沿用OBJ-N方向。
    c = 1.0e6 * r.total_overtime + 1.0e4 * N_late + r.total_distance;
    if(!r.feasible) c += 1.0e12;
    c += 1.0e10 * (double)(M - r.served_count);
  } else {
    // OBJ-N：完全保留既有实验的搜索口径与惩罚，确保旧数据可直接复用。
    c = 1000000.0 * N_late + 100.0 * r.total_overtime + r.total_distance;
    if(!r.feasible) c += 1.0e5;
    c += 500.0 * (double)(M - r.served_count);
  }
  return c;
}


// ★★★ 自适应大邻域搜索 (ALNS) ★★★
//   destroy: 随机移除 / 最差移除 / Shaw 相关移除
//   repair : 贪心插入 / regret-2 插入
//   接受   : 模拟退火;  权重: 按算子近期表现自适应
void ChargeScheduler::solveALNS() {
  const int n = (int)target_vehicles_.size();
  std::mt19937 rng(static_cast<unsigned>(alns_seed_));
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  // ── 初始解：deadline 升序（紧急优先）──
  std::vector<int> cur(n);
  std::iota(cur.begin(), cur.end(), 0);
  std::sort(cur.begin(), cur.end(), [&](int a,int b){
    return target_vehicles_[a].deadline < target_vehicles_[b].deadline; });

  Plan cur_trips, best_trips;
  PlanResult cur_res = simulateOrder(cur, cur_trips);
  PlanResult best_res = cur_res;
  std::vector<int> best = cur;
  best_trips = cur_trips;
  double cur_cost = planCost(cur_res);

  // ── 自适应权重：3 个 destroy × 2 个 repair ──
  const int ND = 3, NR = 2;
  std::vector<double> wd(ND, 1.0), wr(NR, 1.0);
  std::vector<double> sd(ND, 0.0), sr(NR, 0.0);
  std::vector<int>    ud(ND, 0),   ur(NR, 0);
  auto roulette = [&](const std::vector<double> & w)->int{
    double sum = 0.0; for(double x : w) sum += x;
    double r = uni(rng) * sum, acc = 0.0;
    for(size_t i=0;i<w.size();i++) { acc += w[i]; if(r <= acc) return (int)i; }
    return (int)w.size()-1;
  };

  double T = std::max(1.0, alns_T0_ratio_ * std::max(1.0, cur_cost));

  for(int it = 1; it <= alns_iters_; it++) {
    // ---------- destroy ----------
    int q = 1 + (int)(rng() % (size_t)std::max(1, n/4));   // 移除数量
    q = std::min(q, n-1);
    int di = roulette(wd);
    std::vector<int> removed, rest = cur;

    if(di == 0) {                    // ① 随机移除
      for(int k=0;k<q && (int)rest.size()>1;k++) {
        int p = (int)(rng() % rest.size());
        removed.push_back(rest[p]); rest.erase(rest.begin()+p);
      }
    } else if(di == 1) {             // ② 最差移除：绕路增量最大的先走
      std::vector<std::pair<double,int>> gain;
      for(int p=0;p<(int)rest.size();p++) {
        int prev = (p==0) ? 0 : rest[p-1]+1;
        int next = (p+1==(int)rest.size()) ? 0 : rest[p+1]+1;
        double g = D(prev, rest[p]+1) + D(rest[p]+1, next) - D(prev, next);
        gain.push_back({g, rest[p]});
      }
      std::sort(gain.begin(), gain.end(), [](auto&a, auto&b){ return a.first > b.first; });
      for(int k=0;k<q && k<(int)gain.size();k++) removed.push_back(gain[k].second);
      rest.clear();
      for(int v : cur) if(std::find(removed.begin(),removed.end(),v)==removed.end()) rest.push_back(v);
    } else {                         // ③ Shaw 相关移除：与种子最近的一起走
      int seed = cur[(int)(rng() % cur.size())];
      std::vector<std::pair<double,int>> rel;
      for(int v : cur) if(v != seed) rel.push_back({D(seed+1, v+1), v});
      std::sort(rel.begin(), rel.end(), [](auto&a, auto&b){ return a.first < b.first; });
      removed.push_back(seed);
      for(int k=0;k<q-1 && k<(int)rel.size();k++) removed.push_back(rel[k].second);
      rest.clear();
      for(int v : cur) if(std::find(removed.begin(),removed.end(),v)==removed.end()) rest.push_back(v);
    }
    if(removed.empty()) continue;

    // ---------- repair ----------
    //   ★ 修正：按真实目标（planCost）评估插入位置，而非仅看距离增量；
    //     并按截止期升序先插最紧迫的车，保住紧迫度结构。
    int ri = roulette(wr);
    std::sort(removed.begin(), removed.end(), [&](int a,int b){
      return target_vehicles_[a].deadline < target_vehicles_[b].deadline; });

    auto insCost = [&](std::vector<int> & seq, int pos, int cust)->double{
      seq.insert(seq.begin()+pos, cust);
      Plan tp; PlanResult tr = simulateOrder(seq, tp);
      double c = planCost(tr);
      seq.erase(seq.begin()+pos);
      return c;
    };

    if(ri == 0) {                    // ① 贪心插入（按真实目标）
      for(int c : removed) {
        int bp = 0; double bv = 1e18;
        for(int p=0;p<=(int)rest.size();p++) {
          double v = insCost(rest, p, c);
          if(v < bv) { bv = v; bp = p; }
        }
        rest.insert(rest.begin()+bp, c);
      }
    } else {                         // ② regret-2 插入（按真实目标）
      std::vector<int> pool = removed;
      while(!pool.empty()) {
        int pick=-1, pick_pos=0; double best_regret=-1e18;
        for(int c : pool) {
          double b1=1e18, b2=1e18; int p1=0;
          for(int p=0;p<=(int)rest.size();p++) {
            double v = insCost(rest, p, c);
            if(v < b1) { b2=b1; b1=v; p1=p; }
            else if(v < b2) { b2=v; }
          }
          double regret = (b2>1e17?0.0:b2) - (b1>1e17?0.0:b1);
          if(regret > best_regret) { best_regret=regret; pick=c; pick_pos=p1; }
        }
        rest.insert(rest.begin()+pick_pos, pick);
        pool.erase(std::find(pool.begin(), pool.end(), pick));
      }
    }

    // ---------- 评估 + 模拟退火接受 ----------
    Plan cand_trips;
    PlanResult cand_res = simulateOrder(rest, cand_trips);
    double cand_cost = planCost(cand_res);
    double score = 0.0;

    if(isBetter(cand_res, best_res)) {                 // 刷新全局最优
      best = rest; best_res = cand_res; best_trips = cand_trips;
      cur = rest; cur_res = cand_res; cur_cost = cand_cost;
      score = 33.0;
    } else if(cand_cost < cur_cost - 1e-9) {           // 优于当前
      cur = rest; cur_res = cand_res; cur_cost = cand_cost;
      score = 9.0;
    } else if(uni(rng) < std::exp(-(cand_cost - cur_cost) / std::max(T, 1e-9))) {
      cur = rest; cur_res = cand_res; cur_cost = cand_cost;   // 退火接受较差解
      score = 13.0;
    }
    sd[di] += score; ud[di]++;
    sr[ri] += score; ur[ri]++;
    T *= alns_cooling_;

    // ---------- 自适应权重更新 ----------
    if(it % alns_seg_ == 0) {
      const double lambda = 0.7;
      for(int i=0;i<ND;i++) { if(ud[i]) wd[i] = lambda*wd[i] + (1-lambda)*(sd[i]/ud[i]); sd[i]=0; ud[i]=0; wd[i]=std::max(wd[i],0.05); }
      for(int i=0;i<NR;i++) { if(ur[i]) wr[i] = lambda*wr[i] + (1-lambda)*(sr[i]/ur[i]); sr[i]=0; ur[i]=0; wr[i]=std::max(wr[i],0.05); }
    }
  }

  planned_order_ = best;
  final_result_ = simulateOrder(planned_order_, final_plan_); // 最终重放，固定 trip_charger_
  RCLCPP_INFO(get_logger(),
    "[ALNS] objective_order=%s seed=%d iters=%d | N_late=%d(成功率%.1f%%) T_overtime=%.1fmin C_total=%.1fm | 趟数=%d makespan=%.1fmin",
    objectiveOrderLabel().c_str(), alns_seed_, alns_iters_, n - final_result_.on_time_count,
    100.0*final_result_.on_time_count/std::max(1,n), final_result_.total_overtime,
    final_result_.total_distance, final_result_.num_trips, final_result_.total_time);
}

// ── 对照基线：爬山局部搜索（2-swap + 段翻转，仅接受改进解）──
void ChargeScheduler::solveLocalSearch() {
  int n = (int)target_vehicles_.size();
  std::vector<int> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a,int b){
    return target_vehicles_[a].deadline < target_vehicles_[b].deadline; });

  Plan best_trips;
  PlanResult best_res = simulateOrder(order, best_trips);
  std::vector<int> best_order = order;

  std::mt19937 rng(static_cast<unsigned>(alns_seed_));
  int iters = std::min(alns_iters_, n*n*50);
  for(int it=0; it<iters; it++) {
    std::vector<int> cand = best_order;
    int a=rng()%n, b=rng()%n;
    if(a==b) continue;
    if(rng()%2) { std::swap(cand[a], cand[b]); }
    else {
      if(a>b) std::swap(a,b);
      std::reverse(cand.begin()+a, cand.begin()+b+1);
    }
    Plan cand_trips;
    PlanResult cand_res = simulateOrder(cand, cand_trips);
    if(isBetter(cand_res, best_res)) {
      best_res=cand_res; best_order=cand; best_trips=cand_trips;
    }
  }
  planned_order_ = best_order;
  final_result_ = simulateOrder(planned_order_, final_plan_);
  RCLCPP_INFO(get_logger(),
    "[局部搜索] seed=%d iters=%d | N_late=%d(成功率%.1f%%) T_overtime=%.1fmin C_total=%.1fm | 趟数=%d makespan=%.1fmin",
    alns_seed_, iters, n - final_result_.on_time_count,
    100.0*final_result_.on_time_count/std::max(1,n), final_result_.total_overtime,
    final_result_.total_distance, final_result_.num_trips, final_result_.total_time);
}


// ============================================================================
//  ★ HGS：Hybrid Genetic Search（适配自 Vidal 2022, C&OR 140:105643）
//    忠实保留：giant tour + Split 解码、OX 交叉、education 局部搜索、
//              biased fitness（质量秩 + broken-pairs 多样性秩）、
//              可行/不可行双子种群与自适应惩罚、克隆优先淘汰、停滞重启。
//    问题适配：Split = 能量感知动态分趟（simulateOrder）；
//              "不可行" 由容量超载改为【超时】，惩罚项为 wLate·T_overtime；
//              education 在 giant tour 上进行（趟次内生于能量状态，路线非固定），
//              因而未实现依赖固定路线的 SWAP* 邻域。
// ============================================================================
double ChargeScheduler::hgsPhi(const PlanResult & r, double wLate) const {
  const int M = (int)target_vehicles_.size();
  const double N_late = (double)(M - r.on_time_count);
  double c = 0.0;
  if(toverFirst()) {
    // OBJ-T脚本默认不运行HGS；仍保持其教育/排名方向与目标顺序一致。
    c = 1.0e6 * r.total_overtime + 1.0e4 * N_late + r.total_distance;
    if(!r.feasible) c += 1.0e12;
    c += 1.0e10 * (M - r.served_count);
  } else {
    c = 1000000.0 * N_late + wLate * r.total_overtime + r.total_distance;
    if(!r.feasible) c += 1.0e7;
    c += 1.0e5 * (M - r.served_count);
  }
  return c;
}

double ChargeScheduler::hgsBrokenPairs(const std::vector<int> & a, const std::vector<int> & b) const {
  if(a.size() < 2) return 0.0;
  std::set<std::pair<int,int>> A, B;
  for(size_t i=0;i+1<a.size();i++) A.insert({a[i],a[i+1]});
  for(size_t i=0;i+1<b.size();i++) B.insert({b[i],b[i+1]});
  int common=0; for(const auto & e : A) if(B.count(e)) common++;
  return 1.0 - (double)common / std::max<size_t>(1, A.size());
}

std::vector<int> ChargeScheduler::hgsOX(const std::vector<int> & p1,
                                        const std::vector<int> & p2,
                                        std::mt19937 & rng) const {
  int n=(int)p1.size();
  if(n<2) return p1;
  int a=rng()%n, b=rng()%n; if(a>b) std::swap(a,b);
  std::vector<int> child(n,-1);
  std::set<int> taken;
  for(int i=a;i<=b;i++){ child[i]=p1[i]; taken.insert(p1[i]); }   // 继承 P1 片段
  int k=(b+1)%n;
  for(int t=0;t<n;t++){                                           // 从第二切点起环形补齐
    int v=p2[(b+1+t)%n];
    if(taken.count(v)) continue;
    child[k]=v; taken.insert(v); k=(k+1)%n;
  }
  for(int i=0;i<n;i++) if(child[i]<0) { /* 理论不会发生 */ child[i]=p1[i]; }
  return child;
}

// education：在 giant tour 上做 Relocate / Swap / 2-opt，first-improvement
void ChargeScheduler::hgsEducate(std::vector<int> & tour, std::mt19937 & rng, double wLate) {
  int n=(int)tour.size();
  if(n<3) return;
  Plan tp; PlanResult tr = simulateOrder(tour, tp);
  double best = hgsPhi(tr, wLate);
  bool improved=true; int pass=0;
  while(improved && pass++ < 6) {
    improved=false;
    std::vector<int> idx(n); std::iota(idx.begin(), idx.end(), 0);
    std::shuffle(idx.begin(), idx.end(), rng);
    for(int ii : idx) {
      // 粒度限制：只考虑 D 意义下最近的 Γ 个邻居
      std::vector<std::pair<double,int>> nb;
      for(int j=0;j<n;j++) if(j!=ii) nb.push_back({D(tour[ii]+1, tour[j]+1), j});
      std::sort(nb.begin(), nb.end());
      int lim = std::min((int)nb.size(), hgs_gamma_);
      for(int q=0;q<lim && !improved;q++) {
        int jj = nb[q].second;
        for(int mv=0; mv<3 && !improved; mv++) {
          std::vector<int> cand = tour;
          if(mv==0) {                              // Relocate
            int v=cand[ii]; cand.erase(cand.begin()+ii);
            int pos = (jj > ii) ? jj-1 : jj;
            cand.insert(cand.begin()+pos, v);
          } else if(mv==1) {                       // Swap
            std::swap(cand[ii], cand[jj]);
          } else {                                 // 2-opt（段翻转）
            int a=std::min(ii,jj), b=std::max(ii,jj);
            std::reverse(cand.begin()+a, cand.begin()+b+1);
          }
          Plan cp; PlanResult cr = simulateOrder(cand, cp);
          double phi = hgsPhi(cr, wLate);
          if(phi < best - 1e-9) { tour = cand; best = phi; improved = true; }
        }
      }
      if(improved) break;
    }
  }
}

// biased fitness：质量秩 + (1 − nElite/|P|)·多样性秩   [Vidal 2022, Eq.(1)]
void ChargeScheduler::hgsRankFitness(std::vector<HgsIndiv> & sub) const {
  int m=(int)sub.size(); if(m==0) return;
  std::vector<int> byCost(m); std::iota(byCost.begin(), byCost.end(), 0);
  std::sort(byCost.begin(), byCost.end(), [&](int a,int b){ return sub[a].phi < sub[b].phi; });
  std::vector<double> rc(m), rd(m);
  for(int r=0;r<m;r++) rc[byCost[r]] = r;
  // 多样性贡献：到 nClosest 个最相似个体的平均 broken-pairs 距离
  std::vector<double> div(m,0.0);
  for(int i=0;i<m;i++) {
    std::vector<double> ds;
    for(int j=0;j<m;j++) if(i!=j) ds.push_back(hgsBrokenPairs(sub[i].tour, sub[j].tour));
    std::sort(ds.begin(), ds.end());
    int k=std::min((int)ds.size(), hgs_nClosest_);
    double sum=0; for(int t=0;t<k;t++) sum+=ds[t];
    div[i] = k? sum/k : 0.0;
  }
  std::vector<int> byDiv(m); std::iota(byDiv.begin(), byDiv.end(), 0);
  std::sort(byDiv.begin(), byDiv.end(), [&](int a,int b){ return div[a] > div[b]; });  // 越分散秩越小
  for(int r=0;r<m;r++) rd[byDiv[r]] = r;
  double w = 1.0 - (double)hgs_nElite_ / std::max(1, m);
  for(int i=0;i<m;i++) sub[i].bf = rc[i] + w * rd[i];
}

void ChargeScheduler::solveHGS() {
  const int n=(int)target_vehicles_.size();
  std::mt19937 rng((unsigned)alns_seed_);
  double wLate = 20.0;                       // 超时惩罚系数（自适应）
  const int MU=hgs_mu_, LAMBDA=hgs_lambda_;

  auto build=[&](std::vector<int> t)->HgsIndiv{
    HgsIndiv ind; ind.tour=std::move(t);
    ind.res = simulateOrder(ind.tour, ind.trips);
    ind.phi = hgsPhi(ind.res, wLate);
    ind.feasible = (ind.res.total_overtime <= 1e-6) && ind.res.feasible;
    return ind;
  };
  std::vector<HgsIndiv> feas, infeas;
  auto push=[&](HgsIndiv ind){ (ind.feasible? feas: infeas).push_back(std::move(ind)); };

  // 初始种群：4μ 个随机解 + 一个 deadline 升序解，均经 education
  std::vector<int> base(n); std::iota(base.begin(), base.end(), 0);
  std::sort(base.begin(), base.end(), [&](int a,int b){
    return target_vehicles_[a].deadline < target_vehicles_[b].deadline; });
  {
    std::vector<int> t=base; hgsEducate(t,rng,wLate); push(build(t));
    for(int i=0;i<4*MU;i++){
      std::vector<int> r=base; std::shuffle(r.begin(), r.end(), rng);
      hgsEducate(r,rng,wLate); push(build(r));
    }
  }

  HgsIndiv best;
  auto updBest=[&](const HgsIndiv & c){
    if(best.tour.empty() || isBetter(c.res, best.res)) best=c;
  };
  for(const auto& c: feas) updBest(c);
  for(const auto& c: infeas) updBest(c);

  // ★ 锦标赛只读已算好的 bf（排名在每代开头统一更新一次），避免 O(m²) 重复计算
  auto tournament=[&](std::vector<HgsIndiv> & a, std::vector<HgsIndiv> & b)->std::vector<int>{
    std::vector<HgsIndiv> * p = (!a.empty() && (b.empty() || (rng()%2))) ? &a : (!b.empty()? &b : &a);
    if(p->empty()) p = a.empty()? &b : &a;
    int i=rng()%p->size(), j=rng()%p->size();
    return ((*p)[i].bf <= (*p)[j].bf) ? (*p)[i].tour : (*p)[j].tour;   // 返回副本，避免悬垂引用
  };
  auto survivors=[&](std::vector<HgsIndiv> & sub){
    if((int)sub.size() > MU+LAMBDA) hgsRankFitness(sub);
    while((int)sub.size() > MU) {
      // 优先淘汰克隆
      int kill=-1;
      for(size_t i=0;i<sub.size() && kill<0;i++)
        for(size_t j=i+1;j<sub.size();j++)
          if(sub[i].tour==sub[j].tour){ kill=(int)j; break; }
      if(kill<0){ double w=-1; for(size_t i=0;i<sub.size();i++) if(sub[i].bf>w){w=sub[i].bf;kill=(int)i;} }
      sub.erase(sub.begin()+kill);
    }
  };

  int itNoImp=0, feasCount=0, genCount=0;
  auto t0=std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "  [HGS] 初始种群完成: 可行%zu 不可行%zu，开始进化…", feas.size(), infeas.size());
  const double HGS_TIME_LIMIT_MS = 600000.0;   // 安全上限，防止卡死
  while(itNoImp < hgs_nIt_ &&
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count() < HGS_TIME_LIMIT_MS) {
    if(genCount % 10 == 0) {            // 每 10 代更新一次 biased fitness（O(m²) 操作降频）
      hgsRankFitness(feas); hgsRankFitness(infeas);
    }
    std::vector<int> P1 = tournament(feas, infeas);
    std::vector<int> P2 = tournament(feas, infeas);
    std::vector<int> child = hgsOX(P1, P2, rng);
    hgsEducate(child, rng, wLate);                    // Split 在 simulateOrder 内完成
    HgsIndiv C = build(child);

    // 受控不可行探索：50% 概率以 10× 惩罚修复
    if(!C.feasible && (rng()%100) < 50) {
      std::vector<int> rep=C.tour; hgsEducate(rep, rng, wLate*10.0);
      HgsIndiv R = build(rep);
      if(R.feasible || R.phi < C.phi) C = R;
    }
    genCount++; feasCount += C.feasible ? 1 : 0;

    bool imp = (best.tour.empty() || isBetter(C.res, best.res));
    updBest(C);
    itNoImp = imp ? 0 : itNoImp+1;
    push(C);
    if((int)feas.size()   > MU+LAMBDA) survivors(feas);
    if((int)infeas.size() > MU+LAMBDA) survivors(infeas);

    // 惩罚自适应：使可行解比例趋近 ξ_ref
    if(genCount % 100 == 0) {
      double ratio = (double)feasCount / 100.0; feasCount=0;
      if(ratio < hgs_xiRef_)      wLate = std::min(100.0, wLate*1.2);
      else if(ratio > hgs_xiRef_) wLate = std::max(1.0,   wLate*0.85);
    }
  }
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();

  planned_order_ = best.tour;
  final_result_ = simulateOrder(planned_order_, final_plan_); // 最终重放，固定 trip_charger_
  RCLCPP_INFO(get_logger(),
    "[HGS] objective_order=%s seed=%d μ=%d λ=%d Γ=%d | N_late=%d(成功率%.1f%%) T_overtime=%.1fmin C_total=%.1fm | 趟数=%d makespan=%.1fmin | 用时%.0fms",
    objectiveOrderLabel().c_str(), alns_seed_, MU, LAMBDA, hgs_gamma_, n - final_result_.on_time_count,
    100.0*final_result_.on_time_count/std::max(1,n), final_result_.total_overtime,
    final_result_.total_distance, final_result_.num_trips, final_result_.total_time, ms);
}

// 分发
void ChargeScheduler::solveHeuristic() {
  if(heuristic_method_ == "ls")       solveLocalSearch();
  else if(heuristic_method_ == "hgs") solveHGS();
  else                                solveALNS();
}

// 初始解: 按deadline升序(紧急优先)贪心装趟

// 大邻域搜索: 破坏-修复 + 趟内2opt + 趟间迁移

// ============================================================================
//  主调度

// ============================================================================
//  ★ E2耦合消融：DSP/ABS/EUC 仅生成顺序，随后放到【ECD执行一致机动距离】下同K复算
//    输出统一执行口径下的 N_late / T_overtime / C_total / 能量违规
// ============================================================================
void ChargeScheduler::reportCouplingAblation() {
  if(distance_mode_=="maneuver" || final_plan_.empty()) return;
  const int M=(int)target_vehicles_.size();
  std::string source;
  if(distance_mode_=="dsp") source="DSP有向路网";
  else if(distance_mode_=="abstract") source="抽象路网";
  else source="欧氏距离";

  // surrogate 距离只负责生成 giant-tour；统一评估使用 ECD(maneuver) 矩阵、同一个 K 和同一解码器。
  std::vector<int> order=planned_order_;
  if(order.empty()) for(const auto & tr:final_plan_) for(int v:tr) order.push_back(v);

  const auto saved_matrix=dist_matrix_;
  const auto saved_owners=trip_charger_;
  const int saved_rs_node=rs_start_node_;
  const double saved_rs_soc=rs_start_soc_;
  const double saved_rs_time=rs_start_time_;

  dist_matrix_=dist_matrix_real_;
  rs_start_node_=0; rs_start_soc_=-1.0; rs_start_time_=0.0;
  Plan real_plan;
  const PlanResult real_res=simulateOrder(order,real_plan);

  dist_matrix_=saved_matrix;
  trip_charger_=saved_owners;
  rs_start_node_=saved_rs_node; rs_start_soc_=saved_rs_soc; rs_start_time_=saved_rs_time;

  const int nLate=M-real_res.on_time_count;
  RCLCPP_WARN(get_logger(),"════ 耦合消融重放：%s定序 → 同K ECD执行口径 ════",source.c_str());
  RCLCPP_WARN(get_logger(),
    "  【%s仅生成顺序；K=%d、能量、换电、分趟和并行时间线均按ECD执行一致机动距离重新解码】",
    source.c_str(),num_chargers_);
  RCLCPP_WARN(get_logger(),
    "  N_late=%d (按时率 %d/%d = %.1f%%)   T_overtime=%.1fmin   C_total=%.1fm   makespan=%.1fmin   能量违规=%d   objective_order=%s",
    nLate,real_res.on_time_count,M,100.0*real_res.on_time_count/std::max(1,M),
    real_res.total_overtime,real_res.total_distance,real_res.total_time,real_res.energy_violation_count,
    objectiveOrderLabel().c_str());
  RCLCPP_WARN(get_logger(),"  逐辆到达时刻（ECD执行口径、同K解码）：");
  for(int i=0;i<M;i++) {
    const double at=(i<(int)real_res.arrival_time.size())?real_res.arrival_time[i]:-1.0;
    const bool ok=(i<(int)real_res.on_time.size())?real_res.on_time[i]:false;
    RCLCPP_WARN(get_logger(),"     车#%d 到达 %.1f / 限 %.1f %s",i,at,target_vehicles_[i].deadline,
      at<0.0?"✗未服务":(ok?"✓":"✗超时"));
  }
  std::string order_str;
  for(int v:order) order_str+="#"+std::to_string(v)+" ";
  RCLCPP_WARN(get_logger(),"  %s生成的固定访问顺序 = %s",source.c_str(),order_str.c_str());
  RCLCPP_WARN(get_logger(),"════════════════════════════════════════");
}

// ============================================================================
//  多车事件驱动闭环：状态感知并行解码 + 全局剩余任务 ALNS
// ============================================================================
std::string ChargeScheduler::simActionName(SimActionType t) const {
  switch(t) {
    case SimActionType::TARGET_SERVICE: return "target_service";
    case SimActionType::STATION_TURNAROUND: return "station_turnaround";
    case SimActionType::FINAL_RETURN: return "final_return";
  }
  return "unknown";
}

bool ChargeScheduler::subsetBetter(const PlanResult & a, const PlanResult & b) const {
  if(a.feasible != b.feasible) return a.feasible;
  if(a.served_count != b.served_count) return a.served_count > b.served_count;
  const int la = a.served_count - a.on_time_count;
  const int lb = b.served_count - b.on_time_count;
  if(toverFirst()) {
    if(std::abs(a.total_overtime-b.total_overtime)>1e-6)
      return a.total_overtime < b.total_overtime;
    if(la != lb) return la < lb;
  } else {
    if(la != lb) return la < lb;
    if(std::abs(a.total_overtime-b.total_overtime)>1e-6)
      return a.total_overtime < b.total_overtime;
  }
  if(std::abs(a.total_distance-b.total_distance)>1e-6)
    return a.total_distance < b.total_distance;
  return a.total_time < b.total_time;
}

double ChargeScheduler::subsetCost(const PlanResult & r, int pool_size) const {
  const int late = r.served_count - r.on_time_count;
  const int unserved = std::max(0, pool_size-r.served_count);
  double c = 0.0;
  if(toverFirst()) {
    c = 1.0e6*r.total_overtime + 1.0e4*late + r.total_distance;
    c += 1.0e10*unserved;
    if(!r.feasible) c += 1.0e12;
  } else {
    // 与原Fixed/Global实验完全相同。
    c = 1000000.0*late + 100.0*r.total_overtime + r.total_distance;
    c += 1.0e5*unserved;
    if(!r.feasible) c += 1.0e6;
  }
  return c;
}

ChargeScheduler::ParallelDecodedPlan ChargeScheduler::decodeParallelFromState(
  const std::vector<int> & order,
  const std::vector<ChargerDecisionState> & start_states,
  int plan_version)
{
  const int n = static_cast<int>(target_vehicles_.size());
  const int K = std::max(1, num_chargers_);
  const double cap = battery_capacity_kwh_;
  const double low = cap*soc_low_threshold_;
  const double gamma = drive_consume_kwh_per_km_;
  const double INF = 1e17;

  ParallelDecodedPlan out;
  out.queues.assign(K, {});
  out.target_charger.assign(n, -1);
  out.result.arrival_time.assign(n, -1.0);
  out.result.on_time.assign(n, false);
  out.result.feasible = true;

  std::vector<ChargerDecisionState> st(K);
  for(int k=0;k<K;k++) {
    if(k < static_cast<int>(start_states.size())) st[k] = start_states[k];
    else {
      st[k].charger_id = k;
      st[k].available_time = 0.0;
      st[k].node = 0;
      st[k].soc_kwh = cap;
      st[k].next_station_ordinal = 1;
    }
    st[k].charger_id = k;
    if(st[k].soc_kwh < 0.0) st[k].soc_kwh = cap;
    st[k].next_station_ordinal = std::max(1, st[k].next_station_ordinal);
  }
  std::vector<bool> has_trip(K, false);

  auto realD = [&](int a, int b)->double {
    if(a<0 || b<0 || a>=static_cast<int>(dist_matrix_real_.size()) ||
       b>=static_cast<int>(dist_matrix_real_.size())) return INF;
    return dist_matrix_real_[a][b];
  };

  struct Candidate {
    bool valid = false;
    int charger = -1;
    bool use_station = false;
    double finish = 1e18;
    double added_distance = 1e18;
    PlannedAction station;
    PlannedAction target;
  };

  for(int v : order) {
    if(v<0 || v>=n) continue;
    Candidate best;

    for(int k=0;k<K;k++) {
      const auto & s = st[k];

      // 方案A：从该车未来可用状态直接去目标车。
      const double leg = realD(s.node, v+1);
      const double ret = realD(v+1, 0);
      if(leg<INF && ret<INF) {
        const double arr = s.available_time + leg/avg_speed_;
        const double e_drive = leg/1000.0*gamma;
        const double e_charge = chargeEnergyNeed(v, arr);
        const double e_return = ret/1000.0*gamma;
        if(s.soc_kwh-e_drive-e_charge-e_return >= low-1e-9) {
          PlannedAction a;
          a.type = SimActionType::TARGET_SERVICE;
          a.charger_id = k;
          a.target_id = v;
          a.plan_version = plan_version;
          a.start_node = s.node;
          a.end_node = v+1;
          a.planned_start = s.available_time;
          a.planned_arrival = arr;
          a.planned_finish = arr + chargeTimeMin(v, arr);
          a.planned_distance_m = leg;
          a.start_soc_kwh = s.soc_kwh;
          a.end_soc_kwh = s.soc_kwh-e_drive-e_charge;
          if(!best.valid || a.planned_finish<best.finish-1e-9 ||
             (std::abs(a.planned_finish-best.finish)<=1e-9 && leg<best.added_distance)) {
            best.valid = true;
            best.charger = k;
            best.use_station = false;
            best.finish = a.planned_finish;
            best.added_distance = leg;
            best.target = a;
          }
        }
      }

      // 方案B：先返站完成周转，再从驿站出发。
      const double back = realD(s.node, 0);
      const double leg2 = realD(0, v+1);
      if(back<INF && leg2<INF && ret<INF) {
        const double e_back = back/1000.0*gamma;
        const double soc_station = s.soc_kwh-e_back;
        if(soc_station >= low-1e-9) {
          const double station_wait = battery_swap_ ? swap_time_min_ :
            std::max(0.0, cap*soc_depart_threshold_-soc_station)/station_charge_power_kw_*60.0;
          const double station_arr = s.available_time + back/avg_speed_;
          const double dep = station_arr + station_wait;
          const double soc0 = battery_swap_ ? cap : cap*soc_depart_threshold_;
          const double arr = dep + leg2/avg_speed_;
          const double e_drive = leg2/1000.0*gamma;
          const double e_charge = chargeEnergyNeed(v, arr);
          const double e_return = ret/1000.0*gamma;
          if(soc0-e_drive-e_charge-e_return >= low-1e-9) {
            PlannedAction station;
            station.type = SimActionType::STATION_TURNAROUND;
            station.charger_id = k;
            station.station_ordinal = s.next_station_ordinal;
            station.plan_version = plan_version;
            station.start_node = s.node;
            station.end_node = 0;
            station.planned_start = s.available_time;
            station.planned_arrival = station_arr;
            station.planned_finish = dep;
            station.planned_distance_m = back;
            station.start_soc_kwh = s.soc_kwh;
            station.end_soc_kwh = soc0;

            PlannedAction a;
            a.type = SimActionType::TARGET_SERVICE;
            a.charger_id = k;
            a.target_id = v;
            a.plan_version = plan_version;
            a.start_node = 0;
            a.end_node = v+1;
            a.planned_start = dep;
            a.planned_arrival = arr;
            a.planned_finish = arr + chargeTimeMin(v, arr);
            a.planned_distance_m = leg2;
            a.start_soc_kwh = soc0;
            a.end_soc_kwh = soc0-e_drive-e_charge;

            const double add = back+leg2;
            if(!best.valid || a.planned_finish<best.finish-1e-9 ||
               (std::abs(a.planned_finish-best.finish)<=1e-9 && add<best.added_distance)) {
              best.valid = true;
              best.charger = k;
              best.use_station = true;
              best.finish = a.planned_finish;
              best.added_distance = add;
              best.station = station;
              best.target = a;
            }
          }
        }
      }
    }

    if(!best.valid) {
      out.result.feasible = false;
      out.result.energy_violation_count++;
      continue;
    }

    const int k = best.charger;
    if(best.use_station) {
      out.queues[k].push_back(best.station);
      st[k].available_time = best.station.planned_finish;
      st[k].node = 0;
      st[k].soc_kwh = best.station.end_soc_kwh;
      st[k].next_station_ordinal++;
      out.result.total_distance += best.station.planned_distance_m;
      out.result.num_trips++;
      has_trip[k] = true;
    } else if(!has_trip[k]) {
      out.result.num_trips++;
      has_trip[k] = true;
    }

    out.queues[k].push_back(best.target);
    st[k].available_time = best.target.planned_finish;
    st[k].node = best.target.end_node;
    st[k].soc_kwh = best.target.end_soc_kwh;
    out.target_charger[v] = k;

    out.result.arrival_time[v] = best.target.planned_arrival;
    out.result.served_count++;
    const bool ok = best.target.planned_arrival <= target_vehicles_[v].deadline+1e-6;
    out.result.on_time[v] = ok;
    if(ok) out.result.on_time_count++;
    else out.result.total_overtime += best.target.planned_arrival-target_vehicles_[v].deadline;
    out.result.total_distance += best.target.planned_distance_m;
  }

  // 每辆车最终返站。状态感知重调度中，该返站动作尚未开始时可被下一次重调度替换。
  for(int k=0;k<K;k++) {
    if(st[k].node==0) continue;
    const double back = realD(st[k].node, 0);
    if(back>=INF) {
      out.result.feasible = false;
      out.result.energy_violation_count++;
      continue;
    }
    PlannedAction a;
    a.type = SimActionType::FINAL_RETURN;
    a.charger_id = k;
    a.plan_version = plan_version;
    a.start_node = st[k].node;
    a.end_node = 0;
    a.planned_start = st[k].available_time;
    a.planned_arrival = a.planned_start+back/avg_speed_;
    a.planned_finish = a.planned_arrival;
    a.planned_distance_m = back;
    a.start_soc_kwh = st[k].soc_kwh;
    a.end_soc_kwh = st[k].soc_kwh-back/1000.0*gamma;
    if(a.end_soc_kwh<low-1e-9) {
      out.result.feasible = false;
      out.result.energy_violation_count++;
    }
    out.queues[k].push_back(a);
    out.result.total_distance += back;
    st[k].available_time = a.planned_finish;
    st[k].node = 0;
    st[k].soc_kwh = a.end_soc_kwh;
  }

  out.end_states = st;
  out.result.total_time = 0.0;
  for(const auto & s : st) out.result.total_time = std::max(out.result.total_time, s.available_time);
  return out;
}

std::vector<int> ChargeScheduler::optimizeRemainingALNS(
  const std::vector<int> & remaining,
  const std::vector<ChargerDecisionState> & start_states,
  int plan_version,
  ParallelDecodedPlan & best_decoded,
  double & solve_ms)
{
  auto tic = std::chrono::high_resolution_clock::now();
  if(remaining.empty()) {
    best_decoded = decodeParallelFromState({}, start_states, plan_version);
    solve_ms = 0.0;
    return {};
  }

  std::mt19937 rng(static_cast<unsigned>(alns_seed_ + 10007*plan_version));
  std::uniform_real_distribution<double> uni(0.0,1.0);
  std::vector<int> cur = remaining;
  std::sort(cur.begin(), cur.end(), [&](int a,int b){
    return target_vehicles_[a].deadline < target_vehicles_[b].deadline;
  });

  ParallelDecodedPlan cur_dec = decodeParallelFromState(cur, start_states, plan_version);
  ParallelDecodedPlan best_dec = cur_dec;
  std::vector<int> best = cur;
  double cur_cost = subsetCost(cur_dec.result, static_cast<int>(remaining.size()));

  const int ND=3, NR=2;
  std::vector<double> wd(ND,1.0), wr(NR,1.0), sd(ND,0.0), sr(NR,0.0);
  std::vector<int> ud(ND,0), ur(NR,0);
  auto roulette = [&](const std::vector<double> & w)->int {
    double sum=0.0; for(double x:w) sum+=x;
    double r=uni(rng)*sum, acc=0.0;
    for(size_t i=0;i<w.size();i++) { acc+=w[i]; if(r<=acc) return static_cast<int>(i); }
    return static_cast<int>(w.size())-1;
  };
  auto realD = [&](int a,int b)->double {
    if(a<0 || b<0 || a>=static_cast<int>(dist_matrix_real_.size()) ||
       b>=static_cast<int>(dist_matrix_real_.size())) return 1e18;
    return dist_matrix_real_[a][b];
  };
  double T = std::max(1.0, alns_T0_ratio_*std::max(1.0,cur_cost));
  const int iters = std::max(1, reschedule_alns_iters_);

  for(int it=1; it<=iters; ++it) {
    if(cur.size()<2) break;
    int q=1+static_cast<int>(rng()%static_cast<unsigned>(std::max<size_t>(1,cur.size()/4)));
    q=std::min(q, static_cast<int>(cur.size())-1);
    const int di=roulette(wd);
    std::vector<int> removed, rest=cur;

    if(di==0) {
      for(int j=0;j<q && rest.size()>1;j++) {
        int p=static_cast<int>(rng()%rest.size());
        removed.push_back(rest[p]); rest.erase(rest.begin()+p);
      }
    } else if(di==1) {
      std::vector<std::pair<double,int>> gain;
      for(int p=0;p<static_cast<int>(rest.size());p++) {
        int prev=(p==0)?0:rest[p-1]+1;
        int next=(p+1==static_cast<int>(rest.size()))?0:rest[p+1]+1;
        gain.push_back({realD(prev,rest[p]+1)+realD(rest[p]+1,next)-realD(prev,next),rest[p]});
      }
      std::sort(gain.begin(),gain.end(),[](const auto&a,const auto&b){return a.first>b.first;});
      for(int j=0;j<q && j<static_cast<int>(gain.size());j++) removed.push_back(gain[j].second);
      rest.clear();
      for(int v:cur) if(std::find(removed.begin(),removed.end(),v)==removed.end()) rest.push_back(v);
    } else {
      int seed=cur[static_cast<int>(rng()%cur.size())];
      std::vector<std::pair<double,int>> rel;
      for(int v:cur) if(v!=seed) rel.push_back({realD(seed+1,v+1),v});
      std::sort(rel.begin(),rel.end(),[](const auto&a,const auto&b){return a.first<b.first;});
      removed.push_back(seed);
      for(int j=0;j<q-1 && j<static_cast<int>(rel.size());j++) removed.push_back(rel[j].second);
      rest.clear();
      for(int v:cur) if(std::find(removed.begin(),removed.end(),v)==removed.end()) rest.push_back(v);
    }
    if(removed.empty()) continue;

    const int ri=roulette(wr);
    std::sort(removed.begin(),removed.end(),[&](int a,int b){
      return target_vehicles_[a].deadline<target_vehicles_[b].deadline;
    });
    auto insertionCost = [&](std::vector<int> & seq,int pos,int v)->double {
      seq.insert(seq.begin()+pos,v);
      auto d=decodeParallelFromState(seq,start_states,plan_version);
      double c=subsetCost(d.result,static_cast<int>(remaining.size()));
      seq.erase(seq.begin()+pos);
      return c;
    };

    if(ri==0) {
      for(int v:removed) {
        int bp=0; double bv=1e100;
        for(int pos=0;pos<=static_cast<int>(rest.size());pos++) {
          double c=insertionCost(rest,pos,v);
          if(c<bv) {bv=c;bp=pos;}
        }
        rest.insert(rest.begin()+bp,v);
      }
    } else {
      std::vector<int> pool=removed;
      while(!pool.empty()) {
        int pick=pool.front(), pick_pos=0; double best_reg=-1e100;
        for(int v:pool) {
          double b1=1e100,b2=1e100; int p1=0;
          for(int pos=0;pos<=static_cast<int>(rest.size());pos++) {
            double c=insertionCost(rest,pos,v);
            if(c<b1) {b2=b1;b1=c;p1=pos;} else if(c<b2) b2=c;
          }
          double reg=(b2>1e90?b1:b2)-b1;
          if(reg>best_reg) {best_reg=reg;pick=v;pick_pos=p1;}
        }
        rest.insert(rest.begin()+pick_pos,pick);
        pool.erase(std::find(pool.begin(),pool.end(),pick));
      }
    }

    ParallelDecodedPlan cand=decodeParallelFromState(rest,start_states,plan_version);
    const double cand_cost=subsetCost(cand.result,static_cast<int>(remaining.size()));
    double score=0.0;
    if(subsetBetter(cand.result,best_dec.result)) {
      best=rest; best_dec=cand;
      cur=rest; cur_dec=cand; cur_cost=cand_cost; score=33.0;
    } else if(cand_cost<cur_cost-1e-9) {
      cur=rest;cur_dec=cand;cur_cost=cand_cost;score=9.0;
    } else if(uni(rng)<std::exp(-(cand_cost-cur_cost)/std::max(1e-9,T))) {
      cur=rest;cur_dec=cand;cur_cost=cand_cost;score=13.0;
    }
    sd[di]+=score;ud[di]++;sr[ri]+=score;ur[ri]++;
    T*=alns_cooling_;
    if(it%std::max(1,alns_seg_)==0) {
      const double lambda=0.7;
      for(int i=0;i<ND;i++) { if(ud[i]) wd[i]=lambda*wd[i]+(1-lambda)*(sd[i]/ud[i]); wd[i]=std::max(0.05,wd[i]);sd[i]=0;ud[i]=0; }
      for(int i=0;i<NR;i++) { if(ur[i]) wr[i]=lambda*wr[i]+(1-lambda)*(sr[i]/ur[i]); wr[i]=std::max(0.05,wr[i]);sr[i]=0;ur[i]=0; }
    }
  }

  best_decoded=decodeParallelFromState(best,start_states,plan_version);
  auto toc=std::chrono::high_resolution_clock::now();
  solve_ms=std::chrono::duration<double,std::milli>(toc-tic).count();
  return best;
}

ChargeScheduler::ParallelDecodedPlan ChargeScheduler::optimizeFixedAssignmentALNS(
  const std::vector<std::vector<int>> & remaining_by_charger,
  const std::vector<ChargerDecisionState> & start_states,
  int plan_version,
  double & solve_ms)
{
  const int n=static_cast<int>(target_vehicles_.size());
  const int K=static_cast<int>(start_states.size());
  ParallelDecodedPlan merged;
  merged.result.feasible=true;
  merged.result.arrival_time.assign(n,-1.0);
  merged.result.on_time.assign(n,false);
  merged.queues.resize(K);
  merged.end_states.resize(K);
  merged.target_charger.assign(n,-1);
  solve_ms=0.0;
  const int saved_num_chargers=num_chargers_;

  // 固定归属基线：每辆车使用与本文方法相同的 ALNS 和状态感知解码器，
  // 但只优化该车原本拥有的未开始目标。各子问题互不交换目标，因此可以独立求解。
  for(int k=0;k<K;k++) {
    std::vector<int> local_pool;
    if(k<static_cast<int>(remaining_by_charger.size()))
      local_pool=remaining_by_charger[k];

    ChargerDecisionState local_start=start_states[k];
    local_start.charger_id=0;  // 单车子解码器内部索引为0，合并时再映射回真实车辆k。
    std::vector<ChargerDecisionState> one_start{local_start};

    ParallelDecodedPlan local;
    double local_ms=0.0;
    // decodeParallelFromState沿用类成员num_chargers_。固定归属子问题必须临时退化为K=1，
    // 否则其余默认车辆会再次参与分配，破坏“不可跨车”的消融约束。
    num_chargers_=1;
    if(local_pool.empty()) {
      local=decodeParallelFromState({},one_start,plan_version+1000*k);
    } else {
      (void)optimizeRemainingALNS(
        local_pool,one_start,plan_version+1000*k,local,local_ms);
    }
    num_chargers_=saved_num_chargers;
    solve_ms+=local_ms;

    if(!local.queues.empty()) {
      for(auto a:local.queues[0]) {
        a.charger_id=k;
        a.plan_version=plan_version;
        merged.queues[k].push_back(a);
      }
    }
    if(!local.end_states.empty()) {
      merged.end_states[k]=local.end_states[0];
      merged.end_states[k].charger_id=k;
    } else {
      merged.end_states[k]=start_states[k];
    }

    merged.result.feasible = merged.result.feasible && local.result.feasible;
    merged.result.num_trips += local.result.num_trips;
    merged.result.total_distance += local.result.total_distance;
    merged.result.total_time = std::max(merged.result.total_time,local.result.total_time);
    merged.result.on_time_count += local.result.on_time_count;
    merged.result.served_count += local.result.served_count;
    merged.result.energy_violation_count += local.result.energy_violation_count;
    merged.result.total_overtime += local.result.total_overtime;

    for(int v:local_pool) {
      if(v<0 || v>=n) continue;
      merged.target_charger[v]=k;
      if(v<static_cast<int>(local.result.arrival_time.size()))
        merged.result.arrival_time[v]=local.result.arrival_time[v];
      if(v<static_cast<int>(local.result.on_time.size()))
        merged.result.on_time[v]=local.result.on_time[v];
    }
  }
  num_chargers_=saved_num_chargers;
  return merged;
}

ChargeScheduler::DisturbanceSpec ChargeScheduler::makeDisturbanceSpec(
  const ParallelDecodedPlan & initial) const
{
  DisturbanceSpec spec;
  spec.type=disturbance_type_;
  std::transform(spec.type.begin(),spec.type.end(),spec.type.begin(),
    [](unsigned char c){return static_cast<char>(std::tolower(c));});
  spec.delay_min=std::max(0.0,disturbance_delay_min_);

  std::vector<PlannedAction> targets, stations;
  for(const auto & q:initial.queues) for(const auto & a:q) {
    if(a.type==SimActionType::TARGET_SERVICE) targets.push_back(a);
    else if(a.type==SimActionType::STATION_TURNAROUND) stations.push_back(a);
  }
  auto byFinish=[](const PlannedAction&a,const PlannedAction&b){
    if(std::abs(a.planned_finish-b.planned_finish)>1e-9) return a.planned_finish<b.planned_finish;
    if(a.charger_id!=b.charger_id) return a.charger_id<b.charger_id;
    return a.target_id<b.target_id;
  };
  std::sort(targets.begin(),targets.end(),byFinish);
  std::sort(stations.begin(),stations.end(),byFinish);

  if(spec.type=="target" || spec.type=="combined") {
    for(double q:disturbance_target_quantiles_) {
      if(targets.empty()) break;
      q=std::max(0.0,std::min(1.0,q));
      size_t idx=static_cast<size_t>(std::llround(q*static_cast<double>(targets.size()-1)));
      spec.target_ids.insert(targets[idx].target_id);
    }
  }
  if(spec.type=="station" || spec.type=="combined") {
    for(int64_t rank:disturbance_station_ranks_) {
      if(rank<1 || rank>static_cast<int64_t>(stations.size())) continue;
      const auto & a=stations[static_cast<size_t>(rank-1)];
      spec.station_keys.insert({a.charger_id,a.station_ordinal});
    }
  }

  std::ostringstream tg,sg;
  for(int v:spec.target_ids) tg<<v<<",";
  for(const auto & key:spec.station_keys) sg<<"C"<<key.first+1<<"-S"<<key.second<<",";
  RCLCPP_INFO(get_logger(),
    "[扰动清单] type=%s delay=%.1fmin targets={%s} station={%s}",
    spec.type.c_str(),spec.delay_min,tg.str().c_str(),sg.str().c_str());
  return spec;
}

ChargeScheduler::ExecutionSimResult ChargeScheduler::simulateExecution(
  const std::vector<int> & initial_order,
  const ParallelDecodedPlan & initial_plan,
  const DisturbanceSpec & spec,
  bool closed_loop,
  bool fixed_assignment)
{
  (void)initial_order;
  const int n=static_cast<int>(target_vehicles_.size());
  const int K=std::max(1,num_chargers_);
  const double cap=battery_capacity_kwh_;
  const double low=cap*soc_low_threshold_;
  const double gamma=drive_consume_kwh_per_km_;
  const double INF=1e17;
  const char * loop_name = !closed_loop ? "open" :
    (fixed_assignment ? "fixed" : "global");

  ExecutionSimResult out;
  out.result.arrival_time.assign(n,-1.0);
  out.result.on_time.assign(n,false);
  std::vector<bool> served(n,false);
  std::vector<bool> charger_served_any(K,false);
  std::vector<RuntimeCharger> rt(K);
  for(int k=0;k<K;k++) {
    rt[k].charger_id=k;rt[k].actual_time=0.0;rt[k].node=0;rt[k].soc_kwh=cap;
    rt[k].next_station_ordinal=1;
    if(k<static_cast<int>(initial_plan.queues.size())) {
      for(const auto&a:initial_plan.queues[k]) rt[k].future.push_back(a);
    }
  }

  std::priority_queue<SimEvent,std::vector<SimEvent>,SimEventLater> events;
  std::set<int> applied_targets;
  std::set<std::pair<int,int>> applied_stations;
  double actual_distance=0.0;
  double global_time=0.0;
  int energy_viol=0;
  int plan_version=0;

  auto realD=[&](int a,int b)->double {
    if(a<0 || b<0 || a>=static_cast<int>(dist_matrix_real_.size()) ||
       b>=static_cast<int>(dist_matrix_real_.size())) return INF;
    return dist_matrix_real_[a][b];
  };

  std::function<void(int)> dispatchNextAction;
  dispatchNextAction = [&](int k) {
    if(k<0 || k>=K || rt[k].busy || rt[k].future.empty()) return;
    PlannedAction a=rt[k].future.front();
    rt[k].future.pop_front();
    rt[k].active=a;
    rt[k].busy=true;
    rt[k].token++;
    if(a.type==SimActionType::STATION_TURNAROUND)
      rt[k].next_station_ordinal=std::max(rt[k].next_station_ordinal,a.station_ordinal+1);

    const double start=std::max(rt[k].actual_time,a.planned_start);
    SimEvent ev;
    ev.charger_id=k;ev.token=rt[k].token;ev.actual_end_node=rt[k].node;
    ev.actual_end_soc_kwh=rt[k].soc_kwh;

    if(a.type==SimActionType::TARGET_SERVICE) {
      const double leg=realD(rt[k].node,a.target_id+1);
      const double ret=realD(a.target_id+1,0);
      if(leg>=INF || ret>=INF) {
        ev.energy_violation=1;ev.actual_finish=start;ev.actual_arrival=start;
      } else {
        ev.actual_distance_m=leg;
        ev.actual_arrival=start+leg/avg_speed_;
        const double e_drive=leg/1000.0*gamma;
        const double e_charge=chargeEnergyNeed(a.target_id,ev.actual_arrival);
        const double e_ret=ret/1000.0*gamma;
        if(rt[k].soc_kwh-e_drive-e_charge-e_ret<low-1e-9) ev.energy_violation=1;
        ev.actual_end_soc_kwh=std::max(0.0,rt[k].soc_kwh-e_drive-e_charge);
        ev.actual_end_node=a.target_id+1;
        ev.actual_finish=ev.actual_arrival+chargeTimeMin(a.target_id,ev.actual_arrival);
        if(spec.target_ids.count(a.target_id) && !applied_targets.count(a.target_id)) {
          ev.injected_delay_min=spec.delay_min;
          ev.actual_finish+=spec.delay_min;
          applied_targets.insert(a.target_id);
          out.applied_target_disturbances++;
        }
      }
    } else if(a.type==SimActionType::STATION_TURNAROUND) {
      const double back=realD(rt[k].node,0);
      if(back>=INF) {
        ev.energy_violation=1;ev.actual_finish=start;ev.actual_arrival=start;
      } else {
        ev.actual_distance_m=back;
        ev.actual_arrival=start+back/avg_speed_;
        const double soc_station=rt[k].soc_kwh-back/1000.0*gamma;
        if(soc_station<low-1e-9) ev.energy_violation=1;
        const double wait=battery_swap_?swap_time_min_:
          std::max(0.0,cap*soc_depart_threshold_-soc_station)/station_charge_power_kw_*60.0;
        ev.actual_finish=ev.actual_arrival+wait;
        ev.actual_end_node=0;
        ev.actual_end_soc_kwh=battery_swap_?cap:cap*soc_depart_threshold_;
        const std::pair<int,int> key{k,a.station_ordinal};
        if(spec.station_keys.count(key) && !applied_stations.count(key)) {
          ev.injected_delay_min=spec.delay_min;
          ev.actual_finish+=spec.delay_min;
          applied_stations.insert(key);
          out.applied_station_disturbances++;
        }
      }
    } else {
      const double back=realD(rt[k].node,0);
      if(back>=INF) {
        ev.energy_violation=1;ev.actual_finish=start;ev.actual_arrival=start;
      } else {
        ev.actual_distance_m=back;
        ev.actual_arrival=start+back/avg_speed_;
        ev.actual_finish=ev.actual_arrival;
        ev.actual_end_node=0;
        ev.actual_end_soc_kwh=std::max(0.0,rt[k].soc_kwh-back/1000.0*gamma);
        if(ev.actual_end_soc_kwh<low-1e-9) ev.energy_violation=1;
      }
    }
    events.push(ev);
  };

  for(int k=0;k<K;k++) dispatchNextAction(k);

  while(!events.empty()) {
    SimEvent ev=events.top();events.pop();
    if(ev.charger_id<0 || ev.charger_id>=K) continue;
    RuntimeCharger & c=rt[ev.charger_id];
    if(!c.busy || c.token!=ev.token) continue;
    PlannedAction completed=c.active;
    c.busy=false;
    c.actual_time=ev.actual_finish;
    c.node=ev.actual_end_node;
    c.soc_kwh=ev.actual_end_soc_kwh;
    global_time=std::max(global_time,ev.actual_finish);
    actual_distance+=ev.actual_distance_m;
    energy_viol+=ev.energy_violation;

    if(completed.type==SimActionType::TARGET_SERVICE) {
      const int v=completed.target_id;
      if(v>=0 && v<n && !served[v]) {
        served[v]=true;
        out.result.arrival_time[v]=ev.actual_arrival;
        const bool ok=ev.actual_arrival<=target_vehicles_[v].deadline+1e-6;
        out.result.on_time[v]=ok;
        if(!charger_served_any[c.charger_id]) {
          charger_served_any[c.charger_id]=true;
          out.result.num_trips++;
        }
      }
    } else if(completed.type==SimActionType::STATION_TURNAROUND) {
      out.result.num_trips++;
    }

    const double deviation=ev.actual_finish-completed.planned_finish;
    RCLCPP_INFO(get_logger(),
      "[执行事件] loop=%s plan_v=%d C%d type=%s target=%d station_ord=%d planned_finish=%.2f actual_finish=%.2f dev=%+.2f injected=%.1f",
      loop_name,completed.plan_version,c.charger_id+1,
      simActionName(completed.type).c_str(),completed.target_id,completed.station_ordinal,
      completed.planned_finish,ev.actual_finish,deviation,ev.injected_delay_min);

    const bool can_trigger = completed.type==SimActionType::TARGET_SERVICE ||
                             completed.type==SimActionType::STATION_TURNAROUND;
    bool triggered=false;
    if(closed_loop && enable_reschedule_ && can_trigger &&
       deviation+1e-9>=reschedule_threshold_) {
      std::set<int> committed;
      for(const auto & other:rt) {
        if(other.busy && other.active.type==SimActionType::TARGET_SERVICE)
          committed.insert(other.active.target_id);
      }
      std::vector<int> pool;
      for(int v=0;v<n;v++) if(!served[v] && !committed.count(v)) pool.push_back(v);

      if(!pool.empty()) {
        std::vector<int> old_owner(n,-1), old_global;
        std::vector<std::pair<double,int>> old_time;
        for(int k=0;k<K;k++) for(const auto & a:rt[k].future) {
          if(a.type==SimActionType::TARGET_SERVICE && std::find(pool.begin(),pool.end(),a.target_id)!=pool.end()) {
            old_owner[a.target_id]=k;
            old_time.push_back({a.planned_finish,a.target_id});
          }
        }
        std::sort(old_time.begin(),old_time.end());
        for(const auto & x:old_time) old_global.push_back(x.second);

        std::vector<ChargerDecisionState> starts(K);
        for(int k=0;k<K;k++) {
          starts[k].charger_id=k;
          starts[k].next_station_ordinal=rt[k].next_station_ordinal;
          if(rt[k].busy) {
            // 当前已开始动作冻结；调度器只使用其承诺完成状态，不读取隐藏的未来实际扰动。
            starts[k].available_time=std::max(global_time,rt[k].active.planned_finish);
            starts[k].node=rt[k].active.end_node;
            starts[k].soc_kwh=rt[k].active.end_soc_kwh;
          } else {
            starts[k].available_time=global_time;
            starts[k].node=rt[k].node;
            starts[k].soc_kwh=rt[k].soc_kwh;
          }
        }

        ParallelDecodedPlan replanned;
        double solve_ms=0.0;
        const int new_version=++plan_version;
        if(fixed_assignment) {
          std::vector<std::vector<int>> fixed_pools(K);
          for(int v:pool) {
            int owner=(v>=0 && v<n)?old_owner[v]:-1;
            // 正常情况下所有未开始目标都存在于某辆车的future队列。
            // 仅在日志/队列异常时回退到初始归属，仍不允许跨车自由分配。
            if(owner<0 && v>=0 && v<static_cast<int>(initial_plan.target_charger.size()))
              owner=initial_plan.target_charger[v];
            if(owner<0 || owner>=K) owner=0;
            fixed_pools[owner].push_back(v);
          }
          replanned=optimizeFixedAssignmentALNS(
            fixed_pools,starts,new_version,solve_ms);
        } else {
          std::vector<int> new_order=optimizeRemainingALNS(
            pool,starts,new_version,replanned,solve_ms);
          (void)new_order;
        }

        int changed_assign=0;
        for(int v:pool) {
          const int nw=(v<static_cast<int>(replanned.target_charger.size()))?replanned.target_charger[v]:-1;
          if(old_owner[v]>=0 && nw>=0 && old_owner[v]!=nw) changed_assign++;
        }
        std::vector<std::pair<double,int>> new_time;
        for(const auto & q:replanned.queues) for(const auto & a:q)
          if(a.type==SimActionType::TARGET_SERVICE) new_time.push_back({a.planned_finish,a.target_id});
        std::sort(new_time.begin(),new_time.end());
        std::vector<int> new_global;
        for(const auto & x:new_time) new_global.push_back(x.second);
        int changed_order=0;
        const size_t L=std::min(old_global.size(),new_global.size());
        for(size_t i=0;i<L;i++) if(old_global[i]!=new_global[i]) changed_order++;
        changed_order+=static_cast<int>(std::max(old_global.size(),new_global.size())-L);

        for(int k=0;k<K;k++) {
          rt[k].future.clear();
          if(k<static_cast<int>(replanned.queues.size()))
            for(const auto & a:replanned.queues[k]) rt[k].future.push_back(a);
          if(account_reschedule_compute_time_ && !rt[k].busy)
            rt[k].actual_time=std::max(rt[k].actual_time,global_time+solve_ms/60000.0);
        }
        out.reschedule_count++;
        out.total_reschedule_ms+=solve_ms;
        out.changed_assignment_count+=changed_assign;
        out.changed_order_count+=changed_order;
        triggered=true;

        RCLCPP_WARN(get_logger(),
          "[重调度事件] policy=%s #%d t=%.2fmin trigger=C%d/%s dev=%+.2f pool=%zu committed=%zu plan_v=%d solve_ms=%.1f changed_assign=%d changed_order=%d",
          loop_name,out.reschedule_count,global_time,c.charger_id+1,simActionName(completed.type).c_str(),
          deviation,pool.size(),committed.size(),new_version,solve_ms,changed_assign,changed_order);
        for(int k=0;k<K;k++) if(!rt[k].busy) dispatchNextAction(k);
      }
    }
    if(!triggered) dispatchNextAction(c.charger_id);
  }

  out.result.total_distance=actual_distance;
  out.result.total_time=global_time;
  out.result.energy_violation_count=energy_viol;
  out.result.served_count=0;
  out.result.on_time_count=0;
  out.result.total_overtime=0.0;
  for(int v=0;v<n;v++) {
    if(!served[v]) continue;
    out.result.served_count++;
    if(out.result.on_time[v]) out.result.on_time_count++;
    else out.result.total_overtime+=out.result.arrival_time[v]-target_vehicles_[v].deadline;
  }
  out.result.feasible=(energy_viol==0 && out.result.served_count==n);
  return out;
}

void ChargeScheduler::runRescheduleExperiment(double initial_solve_ms) {
  const int n=static_cast<int>(target_vehicles_.size());
  const int K=std::max(1,num_chargers_);
  if(K<2) {
    RCLCPP_ERROR(get_logger(),"[闭环实验] 该模式要求 K>=2，当前 K=%d",K);
    return;
  }
  if(distance_mode_!="maneuver") {
    RCLCPP_WARN(get_logger(),"[闭环实验] 强烈建议 distance_mode=maneuver；当前=%s",distance_mode_.c_str());
  }

  std::string policy=execution_reschedule_policy_;
  std::transform(policy.begin(),policy.end(),policy.begin(),
    [](unsigned char c){return static_cast<char>(std::tolower(c));});
  if(policy=="fixed_assignment" || policy=="fixed-assignment") policy="fixed";
  if(policy=="joint" || policy=="closed") policy="global";
  if(policy!="fixed" && policy!="global" && policy!="all") {
    RCLCPP_WARN(get_logger(),
      "[闭环实验] 未知 execution_reschedule_policy=%s，回退为 global",
      execution_reschedule_policy_.c_str());
    policy="global";
  }

  std::vector<ChargerDecisionState> initial_states(K);
  for(int k=0;k<K;k++) {
    initial_states[k].charger_id=k;
    initial_states[k].available_time=0.0;
    initial_states[k].node=0;
    initial_states[k].soc_kwh=battery_capacity_kwh_;
    initial_states[k].next_station_ordinal=1;
  }
  ParallelDecodedPlan initial=decodeParallelFromState(planned_order_,initial_states,0);
  DisturbanceSpec spec=makeDisturbanceSpec(initial);

  ExecutionSimResult open=simulateExecution(planned_order_,initial,spec,false,false);
  std::vector<std::pair<std::string,ExecutionSimResult>> adaptive;
  if(execution_compare_open_closed_) {
    if(policy=="fixed" || policy=="all")
      adaptive.push_back({"fixed",simulateExecution(planned_order_,initial,spec,true,true)});
    if(policy=="global" || policy=="all")
      adaptive.push_back({"closed",simulateExecution(planned_order_,initial,spec,true,false)});
  }

  auto printResult=[&](const char * mode,const ExecutionSimResult & r) {
    const int nlate=n-r.result.on_time_count;
    RCLCPP_INFO(get_logger(),
      "★闭环实验结果 mode=%s M=%d K=%d disturbance=%s delay=%.1f seed=%d N_late=%d T_overtime=%.3f C_total=%.3f makespan=%.3f served=%d energy_viol=%d reschedules=%d reschedule_ms=%.3f changed_assign=%d changed_order=%d applied_target=%d applied_station=%d initial_solve_ms=%.3f objective_order=%s",
      mode,n,K,spec.type.c_str(),spec.delay_min,alns_seed_,nlate,r.result.total_overtime,
      r.result.total_distance,r.result.total_time,r.result.served_count,
      r.result.energy_violation_count,r.reschedule_count,r.total_reschedule_ms,
      r.changed_assignment_count,r.changed_order_count,r.applied_target_disturbances,
      r.applied_station_disturbances,initial_solve_ms,objectiveOrderLabel().c_str());
  };
  printResult("open",open);
  for(const auto & item:adaptive) {
    printResult(item.first.c_str(),item.second);
    const auto & r=item.second;
    RCLCPP_INFO(get_logger(),
      "★闭环实验收益 policy=%s M=%d K=%d disturbance=%s delay=%.1f seed=%d recovered_late=%d overtime_reduction=%.3f distance_delta=%.3f makespan_reduction=%.3f",
      item.first.c_str(),n,K,spec.type.c_str(),spec.delay_min,alns_seed_,
      (n-open.result.on_time_count)-(n-r.result.on_time_count),
      open.result.total_overtime-r.result.total_overtime,
      r.result.total_distance-open.result.total_distance,
      open.result.total_time-r.result.total_time);
  }

  std_msgs::msg::Bool done;done.data=true;pub_task_complete_->publish(done);
}


// ============================================================================
void ChargeScheduler::runSchedule() {
  if(target_vehicles_.empty()) {
    RCLCPP_WARN(get_logger(), "[Scheduler] 无目标车");
    return;
  }
  int n = (int)target_vehicles_.size();
  RCLCPP_INFO(get_logger(), "======== 开始VRPTW调度 %d 辆车 ========", n);

  auto t0 = std::chrono::high_resolution_clock::now();
  if(n <= 8 && !force_heuristic_) {
    RCLCPP_INFO(get_logger(), "  规模%d≤8 → 精确求解", n);
    solveExact();
  } else {
    RCLCPP_INFO(get_logger(), "  规模%d>8 → 启发式(%s)", n, heuristic_method_.c_str());
    solveHeuristic();
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  double solve_ms = std::chrono::duration<double, std::milli>(t1-t0).count();
  RCLCPP_INFO(get_logger(), "  求解耗时 %.1f ms", solve_ms);

  if(final_plan_.empty()) {
    // 该算例在当前距离矩阵下已确定不可行，禁止定时器每0.5 s重复求解。
    scheduled_ = true;
    RCLCPP_ERROR(get_logger(),
      "[Scheduler] 无可行方案! 已停止本次调度重试，请检查OD可达性和能源约束。");
    return;
  }

  publishResult(final_plan_, final_result_);
  reportCouplingAblation();          // ★ dsp/euclid/abstract 仅定序，随后按ECD执行口径同K重放

  // 多车闭环实验模式：调度节点真实运行，但执行由上层离散事件时钟推进，
  // 不向下层派发任务，也不依赖 RViz。开环/闭环共用同一初始计划与扰动清单。
  if(execution_simulation_mode_) {
    scheduled_ = true;
    runRescheduleExperiment(solve_ms);
    return;
  }

  // ★ 按趟发送: 从第0趟第0辆开始
  current_trip_ = 0;
  trip_vehicle_idx_ = 0;
  waiting_return_ = false;
  dispatching_ = true;
  scheduled_ = true;

  // ★ 闭环重调度：以「任务开始下发」为计时起点（而非第一次到达）
  mission_start_ = now();
  mission_started_ = true;
  actual_cur_node_ = 0;                                  // 从驿站出发
  actual_soc_ = battery_capacity_kwh_;                   // 首趟满电
  actual_distance_m_ = 0.0;
  served_actual_.assign(target_vehicles_.size(), false);
  last_dev_ = 0.0;
  reschedule_count_ = 0;
  active_target_id_ = -1;
  active_actual_arrival_min_ = -1.0;
  active_committed_finish_min_ = -1.0;
  active_target_arrived_ = false;
  active_station_committed_finish_min_ = -1.0;
  online_plan_version_ = 0;
  RCLCPP_INFO(get_logger(), "  ⏱ 任务计时开始 (t=0)");

  publishAllMarkers();
  if(num_chargers_>=2) {
    initializeMultiChargerDispatch();
  } else {
    dispatchNext();
  }
}


void ChargeScheduler::publishResult(const Plan & plan, const PlanResult & res) {
  RCLCPP_INFO(get_logger(), "════════ 调度结果 ════════");
  {
    int M = (int)target_vehicles_.size();
    RCLCPP_INFO(get_logger(),
      "  ★目标(字典序): objective_order=%s | N_late=%d 辆 (按时成功率 %d/%d = %.1f%%)  T_overtime=%.1fmin  C_total=%.1fm",
      objectiveOrderLabel().c_str(), M - res.on_time_count, res.on_time_count, M,
      100.0*res.on_time_count/std::max(1,M), res.total_overtime, res.total_distance);
    RCLCPP_INFO(get_logger(),
      "  附: 趟数=%d | makespan=%.1fmin | 能量可行=%s",
      res.num_trips, res.total_time, res.feasible?"是":"否");
  }
  for(size_t k=0;k<plan.size();k++) {
    const int owner = (k < trip_charger_.size()) ? trip_charger_[k] : 0;
    if(plan[k].empty()) continue;
    std::string s = (num_chargers_ >= 2)
      ? ("  [充电车" + std::to_string(owner+1) + "] 趟" + std::to_string(k+1) + ": 站")
      : ("  趟" + std::to_string(k+1) + ": 站");
    for(int idx : plan[k]) {
      const auto & tv = target_vehicles_[idx];
      char buf[128];
      snprintf(buf,sizeof(buf)," → 车#%d(到%.1f/限%.1f%s)",
        tv.original_index, res.arrival_time[idx], tv.deadline,
        res.on_time[idx]?"✓":"✗超时");
      s += buf;
    }
    s += " → 站";
    if(k < res.charge_drive_battery.size() && res.charge_drive_battery[k])
      s += " [充行驶电池]";
    RCLCPP_INFO(get_logger(), "%s", s.c_str());
  }
  RCLCPP_INFO(get_logger(), "════════════════════════");
}

// ============================================================================
//  触发
// ============================================================================
void ChargeScheduler::onTrigger(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if(!msg->data) return;
  if(!map_loaded_) { RCLCPP_WARN(get_logger(),"地图未就绪"); return; }
  scheduled_ = false;
  runSchedule();
}

void ChargeScheduler::onTimer() {
  if(map_loaded_ && !scheduled_ && !target_vehicles_.empty()) runSchedule();
}

// ============================================================================
//  逐个发送 + 反馈
// ============================================================================
std::unordered_map<std::string,std::string> ChargeScheduler::parseExecutionEvent(
  const std::string & text) const
{
  std::unordered_map<std::string,std::string> out;
  std::string normalized=text;
  std::replace(normalized.begin(),normalized.end(),',',';');
  std::stringstream ss(normalized);
  std::string item;
  auto trim=[](std::string v){
    const auto b=v.find_first_not_of(" \t\r\n\"");
    const auto e=v.find_last_not_of(" \t\r\n\"");
    if(b==std::string::npos) return std::string{};
    return v.substr(b,e-b+1);
  };
  while(std::getline(ss,item,';')) {
    const auto pos=item.find('=');
    if(pos==std::string::npos) continue;
    std::string k=trim(item.substr(0,pos));
    std::string v=trim(item.substr(pos+1));
    if(!k.empty()) out[k]=v;
  }
  return out;
}

void ChargeScheduler::publishAssignmentMeta(
  int charger_id, int target_id, const std::string & event_type,
  double planned_finish_min, int plan_version)
{
  if(!assignment_meta_pub_) return;
  std_msgs::msg::String msg;
  std::ostringstream os;
  os<<"charger_id="<<charger_id
    <<";target_id="<<target_id
    <<";event="<<event_type
    <<";planned_finish_min="<<planned_finish_min
    <<";plan_version="<<plan_version;
  msg.data=os.str();
  assignment_meta_pub_->publish(msg);
}

void ChargeScheduler::onExecutionEvent(const std_msgs::msg::String::ConstSharedPtr msg) {
  const auto f=parseExecutionEvent(msg->data);
  auto get=[&](const std::string & k,const std::string & d=std::string{}){
    auto it=f.find(k);return it==f.end()?d:it->second;
  };
  const std::string event=get("event");
  int charger_id=0,target_id=-1;
  double actual_time=mission_started_?(now()-mission_start_).seconds()/60.0:0.0;
  bool has_soc=false;double soc=0.0;
  try { if(!get("charger_id").empty()) charger_id=std::stoi(get("charger_id")); } catch(...) {}
  try { if(!get("target_id").empty()) target_id=std::stoi(get("target_id")); } catch(...) {}
  try {
    const std::string t=!get("actual_finish_min").empty()?get("actual_finish_min"):get("actual_time_min");
    if(!t.empty()) actual_time=std::stod(t);
  } catch(...) {}
  try { if(!get("actual_soc_kwh").empty()) {soc=std::stod(get("actual_soc_kwh"));has_soc=true;} } catch(...) {}

  if(event=="target_arrived") {
    handleTargetArrivedFeedback(charger_id,target_id,actual_time,has_soc,soc);
  } else if(event=="target_service_completed" || event=="vehicle_served") {
    handleTargetServiceCompletedFeedback(charger_id,target_id,actual_time,has_soc,soc);
  } else if(event=="station_turnaround_completed" || event=="station_served") {
    handleStationTurnaroundCompletedFeedback(charger_id,actual_time,has_soc,soc);
  } else {
    RCLCPP_WARN(get_logger(),"[结构化反馈] 未知/缺少 event: %s",msg->data.c_str());
  }
}

// ============================================================================
// 多充电车真实派发
// 仅复用既有 final_plan_ + trip_charger_ 的分车结果，把单车“发一辆-等反馈-再发下一辆”
// 链复制到独立后缀话题；不修改调度、解码、FSM、规划器或重调度算法。
// ============================================================================
void ChargeScheduler::initializeMultiChargerDispatch() {
  const int K=std::max(1,num_chargers_);
  online_charger_dispatch_.assign(K,OnlineChargerDispatchState{});
  for(int k=0;k<K;k++) online_charger_dispatch_[k].finished=true;

  for(int ti=0;ti<(int)final_plan_.size();ti++) {
    int owner=(ti<(int)trip_charger_.size())?trip_charger_[ti]:0;
    if(owner<0 || owner>=K) {
      RCLCPP_WARN(get_logger(),
        "[多车派发] 趟%d的owner=%d越界，回退到充电车1",ti+1,owner+1);
      owner=0;
    }
    if(final_plan_[ti].empty()) continue;
    online_charger_dispatch_[owner].trip_indices.push_back(ti);
    online_charger_dispatch_[owner].finished=false;
  }

  for(int k=0;k<K;k++) {
    const auto & st=online_charger_dispatch_[k];
    if(st.trip_indices.empty()) {
      RCLCPP_INFO(get_logger(),
        "[多车派发] 充电车%d未分配任务，不向后缀_%d发布顺序",k+1,k+1);
      continue;
    }
    size_t target_count=0;
    for(int ti:st.trip_indices) target_count+=final_plan_[ti].size();
    RCLCPP_INFO(get_logger(),
      "[多车派发] 充电车%d分配%zu趟、%zu辆目标车，开始独立逐任务派发",
      k+1,st.trip_indices.size(),target_count);
    dispatchNextForCharger(k);
  }
}

bool ChargeScheduler::allUsedChargersFinished() const {
  bool has_used=false;
  for(const auto & st:online_charger_dispatch_) {
    if(st.trip_indices.empty()) continue;
    has_used=true;
    if(!st.finished) return false;
  }
  return has_used;
}

void ChargeScheduler::dispatchNextForCharger(int charger_id) {
  if(charger_id<0 || charger_id>=(int)online_charger_dispatch_.size()) return;
  auto & st=online_charger_dispatch_[charger_id];
  if(st.finished || st.trip_indices.empty()) return;

  if(st.trip_pos>=(int)st.trip_indices.size()) {
    st.finished=true;
    st.active_target_id=-1;
    std_msgs::msg::Bool done;done.data=true;
    pub_task_completes_by_charger_[charger_id]->publish(done);
    RCLCPP_INFO(get_logger(),
      "[多车派发] 充电车%d全部已分配趟完成，已发布 task_complete_%d",
      charger_id+1,charger_id+1);
    if(allUsedChargersFinished()) {
      dispatching_=false;
      RCLCPP_INFO(get_logger(),"════ 所有已使用充电车任务完成 ════");
      publishAllMarkers();
    }
    return;
  }

  const int trip_index=st.trip_indices[st.trip_pos];
  if(trip_index<0 || trip_index>=(int)final_plan_.size()) {
    RCLCPP_ERROR(get_logger(),"[多车派发] 充电车%d趟索引%d无效",charger_id+1,trip_index);
    st.finished=true;
    return;
  }
  const Trip & trip=final_plan_[trip_index];
  if(st.vehicle_pos>=(int)trip.size()) {
    sendReturnToStationForCharger(charger_id);
    return;
  }

  const int idx=trip[st.vehicle_pos];
  const auto & tv=target_vehicles_[idx];
  PredictedObjects msg;
  msg.header.stamp=now();
  msg.header.frame_id="map";
  autoware_auto_perception_msgs::msg::PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position=tv.pos;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w=1.0;
  autoware_auto_perception_msgs::msg::ObjectClassification cls;
  cls.label=1;cls.probability=1.0;
  obj.classification.push_back(cls);
  msg.objects.push_back(obj);
  pub_sequences_by_charger_[charger_id]->publish(msg);

  const double pred_arr=(idx<(int)final_result_.arrival_time.size() && final_result_.arrival_time[idx]>=0.0)
    ?final_result_.arrival_time[idx]:0.0;
  const double pred_finish=pred_arr+chargeTimeMin(idx,pred_arr);
  st.active_target_id=idx;
  publishAssignmentMeta(charger_id,idx,"target_assignment",pred_finish,online_plan_version_);

  RCLCPP_INFO(get_logger(),
    "  → [充电车%d] 第%d/%zu趟 第%d/%zu辆: 车#%d (%.2f,%.2f) | topic后缀=_%d | 承诺完成=%.2fmin",
    charger_id+1,st.trip_pos+1,st.trip_indices.size(),st.vehicle_pos+1,trip.size(),
    tv.original_index,tv.pos.x,tv.pos.y,charger_id+1,pred_finish);
  publishAllMarkers();
}

void ChargeScheduler::sendReturnToStationForCharger(int charger_id) {
  if(charger_id<0 || charger_id>=(int)online_charger_dispatch_.size()) return;
  auto & st=online_charger_dispatch_[charger_id];
  if(st.waiting_return || st.finished) return;
  st.waiting_return=true;

  geometry_msgs::msg::PoseStamped rg;
  rg.header.stamp=now();
  rg.header.frame_id="map";
  rg.pose.position=charging_station_;
  rg.pose.orientation.w=1.0;
  pub_return_goals_by_charger_[charger_id]->publish(rg);
  publishAssignmentMeta(charger_id,-1,"station_turnaround_assignment",-1.0,online_plan_version_);
  RCLCPP_INFO(get_logger(),
    "  → [充电车%d] 当前趟完成，已发布 return_goal_%d，等待 arrived_station_%d",
    charger_id+1,charger_id+1,charger_id+1);
}

void ChargeScheduler::startNextTripForCharger(int charger_id) {
  if(charger_id<0 || charger_id>=(int)online_charger_dispatch_.size()) return;
  auto & st=online_charger_dispatch_[charger_id];
  st.waiting_return=false;
  st.trip_pos++;
  st.vehicle_pos=0;
  st.active_target_id=-1;
  dispatchNextForCharger(charger_id);
}

void ChargeScheduler::onVehicleServedForCharger(
  int charger_id,const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if(charger_id==0 && num_chargers_==1) {
    onVehicleServed(msg);
    return;
  }
  if(prefer_structured_feedback_ || !msg->data || !dispatching_) return;
  if(charger_id<0 || charger_id>=(int)online_charger_dispatch_.size()) return;
  auto & st=online_charger_dispatch_[charger_id];
  if(st.finished || st.active_target_id<0) return;

  const int target_id=st.active_target_id;
  if(target_id>=0 && target_id<(int)served_actual_.size()) served_actual_[target_id]=true;
  RCLCPP_INFO(get_logger(),
    "  ✓ [充电车%d] 车#%d服务完成，继续发送该车下一任务",
    charger_id+1,target_id);
  st.active_target_id=-1;
  st.vehicle_pos++;
  dispatchNextForCharger(charger_id);
}

void ChargeScheduler::onArrivedTargetForCharger(
  int charger_id,const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if(charger_id==0 && num_chargers_==1) {
    onArrivedTarget(msg);
    return;
  }
  if(prefer_structured_feedback_ || !msg->data || !dispatching_) return;
  if(charger_id<0 || charger_id>=(int)online_charger_dispatch_.size()) return;
  const auto & st=online_charger_dispatch_[charger_id];
  if(st.finished || st.active_target_id<0) return;
  RCLCPP_INFO(get_logger(),
    "  ⏱ [充电车%d] 已到达车#%d停靠点",charger_id+1,st.active_target_id);
}

void ChargeScheduler::onArrivedStationForCharger(
  int charger_id,const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if(charger_id==0 && num_chargers_==1) {
    onArrivedStation(msg);
    return;
  }
  if(prefer_structured_feedback_ || !msg->data || !dispatching_) return;
  if(charger_id<0 || charger_id>=(int)online_charger_dispatch_.size()) return;
  auto & st=online_charger_dispatch_[charger_id];
  if(st.finished || !st.waiting_return) return;
  RCLCPP_INFO(get_logger(),
    "  ✓ [充电车%d] 已回站并完成周转，开始该车下一趟",charger_id+1);
  startNextTripForCharger(charger_id);
}

// 发送当前趟内的下一辆车
void ChargeScheduler::dispatchNext() {
  if(current_trip_ >= (int)final_plan_.size()) {
    RCLCPP_INFO(get_logger(), "════ 所有趟完成! 发任务结束信号 ════");
    dispatching_ = false;
    std_msgs::msg::Bool done; done.data = true;
    pub_task_complete_->publish(done);
    publishAllMarkers();
    return;
  }

  const Trip & trip = final_plan_[current_trip_];
  if(trip_vehicle_idx_ >= (int)trip.size()) {
    RCLCPP_INFO(get_logger(), "════ 第%d趟充完%zu辆 → 充电车回站换电池 ════",
      current_trip_+1, trip.size());
    sendReturnToStation();
    return;
  }

  int idx = trip[trip_vehicle_idx_];
  const auto & tv = target_vehicles_[idx];
  PredictedObjects msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  autoware_auto_perception_msgs::msg::PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position = tv.pos;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  autoware_auto_perception_msgs::msg::ObjectClassification cls;
  cls.label = 1; cls.probability = 1.0;
  obj.classification.push_back(cls);
  msg.objects.push_back(obj);
  pub_sequence_->publish(msg);

  const double pred_arr=(idx<(int)final_result_.arrival_time.size() && final_result_.arrival_time[idx]>=0.0)
    ?final_result_.arrival_time[idx]:0.0;
  const double pred_finish=pred_arr+chargeTimeMin(idx,pred_arr);
  active_target_id_=idx;
  active_target_arrived_=false;
  active_actual_arrival_min_=-1.0;
  active_committed_finish_min_=pred_finish;
  publishAssignmentMeta(0,idx,"target_assignment",pred_finish,online_plan_version_);

  RCLCPP_INFO(get_logger(), "  → 趟%d 第%d辆: 车#%d [%s] (%.2f,%.2f) 容量%.2fkWh 剩余%.0f%% | 承诺完成=%.2fmin plan_v=%d",
    current_trip_+1, trip_vehicle_idx_+1,
    tv.original_index, tv.vehicle_type.c_str(), tv.pos.x, tv.pos.y,
    tv.capacity_kwh, tv.remaining_battery,pred_finish,online_plan_version_);
  publishAllMarkers();
}

// 发返程目标。旧 arrived_station Bool 为兼容接口，仍按“站端周转已完成”处理；
// 未来实车推荐在换电/补能真正完成后发布结构化 station_turnaround_completed。
void ChargeScheduler::sendReturnToStation() {
  waiting_return_ = true;
  geometry_msgs::msg::PoseStamped rg;
  rg.header.stamp = now();
  rg.header.frame_id = "map";
  rg.pose.position = charging_station_;
  rg.pose.orientation.w = 1.0;
  pub_return_goal_->publish(rg);

  const double t_now=mission_started_?(now()-mission_start_).seconds()/60.0:0.0;
  double back=D(actual_cur_node_,0);if(back>=1e17) back=0.0;
  active_station_committed_finish_min_=t_now+back/avg_speed_+
    (battery_swap_?swap_time_min_:0.0);
  publishAssignmentMeta(0,-1,"station_turnaround_assignment",
    active_station_committed_finish_min_,online_plan_version_);
  RCLCPP_INFO(get_logger(), "  → 回站目标(站%.2f,%.2f)已发布，承诺周转完成=%.2fmin",
    charging_station_.x, charging_station_.y,active_station_committed_finish_min_);
}

void ChargeScheduler::handleTargetArrivedFeedback(
  int charger_id,int target_id,double actual_time_min,bool has_soc,double actual_soc_kwh)
{
  if(charger_id!=0) {
    RCLCPP_INFO(get_logger(),
      "[结构化反馈] 收到充电车%d target_arrived(target=%d,t=%.2f)；接口已保留，多实车派发器后续接入",
      charger_id+1,target_id,actual_time_min);
    return;
  }
  if(!dispatching_ || !mission_started_) return;
  if(target_id<0) target_id=active_target_id_;
  if(target_id<0 || target_id>=(int)target_vehicles_.size()) return;
  if(active_target_arrived_ && active_target_id_==target_id) return;

  double pred_arr=(target_id<(int)final_result_.arrival_time.size())?
    final_result_.arrival_time[target_id]:actual_time_min;
  last_dev_=actual_time_min-pred_arr;
  double seg=D(actual_cur_node_,target_id+1);
  if(seg<1e17) {
    actual_distance_m_+=seg;
    if(actual_soc_<0.0) actual_soc_=battery_capacity_kwh_;
    actual_soc_=std::max(0.0,actual_soc_-seg/1000.0*drive_consume_kwh_per_km_);
  }
  if(has_soc) actual_soc_=actual_soc_kwh;
  actual_cur_node_=target_id+1;
  active_target_id_=target_id;
  active_target_arrived_=true;
  active_actual_arrival_min_=actual_time_min;
  active_committed_finish_min_=pred_arr+chargeTimeMin(target_id,pred_arr);
  if((int)final_result_.arrival_time.size()==(int)target_vehicles_.size())
    final_result_.arrival_time[target_id]=actual_time_min;
  if((int)final_result_.on_time.size()==(int)target_vehicles_.size())
    final_result_.on_time[target_id]=actual_time_min<=target_vehicles_[target_id].deadline+1e-6;

  RCLCPP_INFO(get_logger(),
    "  ⏱ 到达车#%d: actual=%.2f predicted=%.2f arrival_dev=%+.2f | committed_service_finish=%.2f | deadline=%.1f %s",
    target_id,actual_time_min,pred_arr,last_dev_,active_committed_finish_min_,
    target_vehicles_[target_id].deadline,
    actual_time_min<=target_vehicles_[target_id].deadline+1e-6?"✓":"✗超时");
}

void ChargeScheduler::handleTargetServiceCompletedFeedback(
  int charger_id,int target_id,double actual_time_min,bool has_soc,double actual_soc_kwh)
{
  if(charger_id!=0) {
    RCLCPP_INFO(get_logger(),
      "[结构化反馈] 收到充电车%d target_service_completed(target=%d,t=%.2f)；多车事件字段已兼容",
      charger_id+1,target_id,actual_time_min);
    return;
  }
  if(!dispatching_ || !mission_started_) return;
  if(target_id<0) target_id=active_target_id_;
  if(target_id<0 || target_id>=(int)target_vehicles_.size()) return;
  if((int)served_actual_.size()!=(int)target_vehicles_.size())
    served_actual_.assign(target_vehicles_.size(),false);
  if(served_actual_[target_id]) return; // 防止Bool与结构化事件重复上报

  if(!active_target_arrived_ || active_target_id_!=target_id) {
    double pred=(target_id<(int)final_result_.arrival_time.size())?final_result_.arrival_time[target_id]:actual_time_min;
    handleTargetArrivedFeedback(0,target_id,pred,false,0.0);
  }
  if(has_soc) actual_soc_=actual_soc_kwh;
  else {
    if(actual_soc_<0.0) actual_soc_=battery_capacity_kwh_;
    actual_soc_=std::max(0.0,actual_soc_-chargeEnergyNeed(target_id,
      std::max(0.0,active_actual_arrival_min_)));
  }
  served_actual_[target_id]=true;
  last_dev_=actual_time_min-active_committed_finish_min_;
  RCLCPP_INFO(get_logger(),
    "  ✓ 车#%d 服务完成: actual_finish=%.2f committed_finish=%.2f finish_dev=%+.2f SoC=%.2fkWh",
    target_id,actual_time_min,active_committed_finish_min_,last_dev_,actual_soc_);

  if(enable_reschedule_ && last_dev_+1e-9>=reschedule_threshold_) {
    RCLCPP_WARN(get_logger(),
      "  ⚠ 完成偏差 %.2fmin >= 阈值 %.1fmin → 在服务完成后重调度",
      last_dev_,reschedule_threshold_);
    maybeReschedule(actual_time_min);
  }

  active_target_id_=-1;active_target_arrived_=false;
  active_actual_arrival_min_=-1.0;active_committed_finish_min_=-1.0;
  trip_vehicle_idx_++;
  dispatchNext();
}

void ChargeScheduler::handleStationTurnaroundCompletedFeedback(
  int charger_id,double actual_time_min,bool has_soc,double actual_soc_kwh)
{
  if(charger_id!=0) {
    RCLCPP_INFO(get_logger(),
      "[结构化反馈] 收到充电车%d station_turnaround_completed(t=%.2f)；多车事件字段已兼容",
      charger_id+1,actual_time_min);
    return;
  }
  if(!waiting_return_) return;

  if(actual_cur_node_!=0) {
    const double back=D(actual_cur_node_,0);
    if(back<1e17) {
      actual_distance_m_+=back;
      if(actual_soc_<0.0) actual_soc_=battery_capacity_kwh_;
      actual_soc_=std::max(0.0,actual_soc_-back/1000.0*drive_consume_kwh_per_km_);
    }
  }
  actual_cur_node_=0;
  actual_soc_=has_soc?actual_soc_kwh:
    (battery_swap_?battery_capacity_kwh_:battery_capacity_kwh_*soc_depart_threshold_);
  last_dev_=(active_station_committed_finish_min_>=0.0)?
    actual_time_min-active_station_committed_finish_min_:0.0;
  RCLCPP_INFO(get_logger(),
    "  ✓ 站端周转完成: actual=%.2f committed=%.2f dev=%+.2f SoC=%.2fkWh",
    actual_time_min,active_station_committed_finish_min_,last_dev_,actual_soc_);

  if(enable_reschedule_ && last_dev_+1e-9>=reschedule_threshold_) {
    RCLCPP_WARN(get_logger(),
      "  ⚠ 站端周转偏差 %.2fmin >= 阈值 %.1fmin → 在周转完成后重调度",
      last_dev_,reschedule_threshold_);
    maybeReschedule(actual_time_min);
  }
  waiting_return_=false;
  active_station_committed_finish_min_=-1.0;
  startNextTrip();
}

void ChargeScheduler::onArrivedStation(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if(prefer_structured_feedback_ || !msg->data) return;
  const double t=mission_started_?(now()-mission_start_).seconds()/60.0:0.0;
  handleStationTurnaroundCompletedFeedback(0,t,false,0.0);
}

void ChargeScheduler::startNextTrip() {
  current_trip_++;
  trip_vehicle_idx_=0;
  if(current_trip_ >= (int)final_plan_.size()) {
    RCLCPP_INFO(get_logger(), "════ 全部趟完成! 发任务结束信号 ════");
    dispatching_=false;
    std_msgs::msg::Bool done;done.data=true;pub_task_complete_->publish(done);
    publishAllMarkers();
    return;
  }
  RCLCPP_INFO(get_logger(), "════ 开始第%d趟 ════",current_trip_+1);
  dispatchNext();
}

void ChargeScheduler::onVehicleServed(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if(prefer_structured_feedback_ || !msg->data) return;
  const double t=mission_started_?(now()-mission_start_).seconds()/60.0:0.0;
  handleTargetServiceCompletedFeedback(0,active_target_id_,t,false,0.0);
}

void ChargeScheduler::onArrivedTarget(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if(prefer_structured_feedback_ || !msg->data) return;
  const double t=mission_started_?(now()-mission_start_).seconds()/60.0:0.0;
  handleTargetArrivedFeedback(0,active_target_id_,t,false,0.0);
}

void ChargeScheduler::maybeReschedule(double t_now_min) {
  const int n=(int)target_vehicles_.size();
  std::vector<int> remaining;
  for(int i=0;i<n;i++) if(i<(int)served_actual_.size() && !served_actual_[i]) remaining.push_back(i);
  if(remaining.size()<2) {
    RCLCPP_INFO(get_logger(),"  剩余车辆不足，跳过重调度");
    return;
  }
  if(num_chargers_!=1) {
    RCLCPP_WARN(get_logger(),
      "  当前ROS逐任务派发器仍是单执行链；K=%d的真实多车请使用结构化反馈+未来多车派发器。批量多车闭环实验已由事件仿真模式实现。",
      num_chargers_);
    return;
  }

  const bool trigger_after_station=waiting_return_ && actual_cur_node_==0;
  rs_start_node_=actual_cur_node_;
  rs_start_soc_=actual_soc_;
  rs_start_time_=t_now_min;

  ChargerDecisionState st;
  st.charger_id=0;st.available_time=t_now_min;st.node=actual_cur_node_;
  st.soc_kwh=(actual_soc_<0.0)?battery_capacity_kwh_:actual_soc_;
  st.next_station_ordinal=1;
  ParallelDecodedPlan decoded;
  double solve_ms=0.0;
  const int new_version=online_plan_version_+1;
  std::vector<int> new_order=optimizeRemainingALNS(
    remaining,std::vector<ChargerDecisionState>{st},new_version,decoded,solve_ms);

  Plan new_plan;
  Trip cur;
  bool first_requires_station=false;
  bool saw_target=false;
  if(!decoded.queues.empty()) {
    for(const auto & a:decoded.queues[0]) {
      if(a.type==SimActionType::STATION_TURNAROUND) {
        if(!saw_target && cur.empty()) first_requires_station=true;
        if(!cur.empty()) {new_plan.push_back(cur);cur.clear();}
      } else if(a.type==SimActionType::TARGET_SERVICE) {
        cur.push_back(a.target_id);saw_target=true;
      }
    }
  }
  if(!cur.empty()) new_plan.push_back(cur);
  if(new_plan.empty()) {
    RCLCPP_WARN(get_logger(),"  重调度未生成剩余任务方案，保留旧计划");
    rs_start_node_=0;rs_start_soc_=-1.0;rs_start_time_=0.0;
    return;
  }

  Plan rebuilt;
  if(trigger_after_station) {
    // 当前趟已经完整结束并完成站端周转；所有新任务从下一趟开始。
    for(int k=0;k<=current_trip_ && k<(int)final_plan_.size();k++) rebuilt.push_back(final_plan_[k]);
    for(const auto & tr:new_plan) if(!tr.empty()) rebuilt.push_back(tr);
  } else {
    for(int k=0;k<current_trip_ && k<(int)final_plan_.size();k++) rebuilt.push_back(final_plan_[k]);
    Trip continued;
    if(current_trip_<(int)final_plan_.size()) {
      const Trip & old=final_plan_[current_trip_];
      const int keep=std::min((int)old.size(),trip_vehicle_idx_+1);
      continued.insert(continued.end(),old.begin(),old.begin()+keep);
    }
    size_t next_trip=0;
    if(!first_requires_station && !new_plan.empty()) {
      continued.insert(continued.end(),new_plan[0].begin(),new_plan[0].end());
      next_trip=1;
    }
    if(!continued.empty()) rebuilt.push_back(continued);
    for(size_t k=next_trip;k<new_plan.size();k++) if(!new_plan[k].empty()) rebuilt.push_back(new_plan[k]);
  }
  final_plan_=std::move(rebuilt);
  trip_charger_.assign(final_plan_.size(),0);
  planned_order_=new_order;
  online_plan_version_=new_version;

  PlanResult merged=final_result_;
  if((int)merged.arrival_time.size()!=n) merged.arrival_time.assign(n,-1.0);
  if((int)merged.on_time.size()!=n) merged.on_time.assign(n,false);
  for(int v:remaining) {
    if(v<(int)decoded.result.arrival_time.size()) merged.arrival_time[v]=decoded.result.arrival_time[v];
    if(v<(int)decoded.result.on_time.size()) merged.on_time[v]=decoded.result.on_time[v];
  }
  merged.feasible=decoded.result.feasible;
  merged.num_trips=(int)final_plan_.size();
  merged.total_time=decoded.result.total_time;
  merged.total_distance=actual_distance_m_+decoded.result.total_distance;
  merged.energy_violation_count=decoded.result.energy_violation_count;
  merged.served_count=0;merged.on_time_count=0;merged.total_overtime=0.0;
  for(int i=0;i<n;i++) {
    if(merged.arrival_time[i]<0.0) continue;
    merged.served_count++;
    const bool ok=merged.arrival_time[i]<=target_vehicles_[i].deadline+1e-6;
    merged.on_time[i]=ok;
    if(ok) merged.on_time_count++;
    else merged.total_overtime+=merged.arrival_time[i]-target_vehicles_[i].deadline;
  }
  final_result_=std::move(merged);
  reschedule_count_++;

  RCLCPP_INFO(get_logger(),
    "  ♻ 在线ALNS重调度#%d: t=%.2f remaining=%zu plan_v=%d solve_ms=%.1f N_late(rem)=%d T_over(rem)=%.1f C_future=%.1f",
    reschedule_count_,t_now_min,remaining.size(),online_plan_version_,solve_ms,
    decoded.result.served_count-decoded.result.on_time_count,
    decoded.result.total_overtime,decoded.result.total_distance);

  rs_start_node_=0;rs_start_soc_=-1.0;rs_start_time_=0.0;
  publishAllMarkers();
}

// ============================================================================
//  可视化
// ============================================================================
void ChargeScheduler::publishAllMarkers() {
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  // 遍历所有趟的所有车, 按 已服务(灰)/当前趟当前辆(绿)/待服务(红) 上色
  int marker_id = 0;
  for(size_t ti=0; ti<final_plan_.size(); ti++) {
    for(size_t vi=0; vi<final_plan_[ti].size(); vi++) {
      const auto & tv = target_vehicles_[final_plan_[ti][vi]];
      float r,g,b;
      if((int)ti < current_trip_ ||
         ((int)ti == current_trip_ && (int)vi < trip_vehicle_idx_)) {
        r=0.5f; g=0.5f; b=0.5f;  // 已服务
      } else if((int)ti == current_trip_ && (int)vi == trip_vehicle_idx_) {
        r=0.0f; g=1.0f; b=0.0f;  // 当前
      } else {
        r=1.0f; g=0.0f; b=0.0f;  // 待服务
      }

      visualization_msgs::msg::Marker m;
      m.header.frame_id="map"; m.header.stamp=now();
      m.ns="target_vehicles"; m.id=marker_id;
      m.type=visualization_msgs::msg::Marker::SPHERE;
      m.action=visualization_msgs::msg::Marker::ADD;
      m.pose.position=tv.pos; m.pose.position.z=1.0;
      m.pose.orientation.w=1.0;
      m.scale.x=m.scale.y=m.scale.z=2.0;
      m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=0.9f;
      arr.markers.push_back(m);

      visualization_msgs::msg::Marker t;
      t.header.frame_id="map"; t.header.stamp=now();
      t.ns="target_labels"; t.id=marker_id;
      t.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action=visualization_msgs::msg::Marker::ADD;
      t.pose.position=tv.pos; t.pose.position.z=3.5;
      t.pose.orientation.w=1.0;
      t.scale.z=1.5;
      t.color.r=1.0f; t.color.g=1.0f; t.color.b=1.0f; t.color.a=1.0f;
      char lbl[64];
      snprintf(lbl,sizeof(lbl),"T%zu-%zu bat:%.0f%%", ti+1, vi+1, tv.remaining_battery);
      t.text = lbl;
      arr.markers.push_back(t);
      marker_id++;
    }
  }
  if(false) {  // 旧代码保留结构(不执行)
  for(size_t k=0;k<planned_order_.size();k++) {
    const auto & tv = target_vehicles_[planned_order_[k]];
    float r,g,b;
    if((int)k < dispatch_index_)       { r=0.5f; g=0.5f; b=0.5f; }
    else if((int)k == dispatch_index_) { r=0.0f; g=1.0f; b=0.0f; }
    else                               { r=1.0f; g=0.0f; b=0.0f; }

    visualization_msgs::msg::Marker m;
    m.header.frame_id="map"; m.header.stamp=now();
    m.ns="target_vehicles"; m.id=(int)k;
    m.type=visualization_msgs::msg::Marker::SPHERE;
    m.action=visualization_msgs::msg::Marker::ADD;
    m.pose.position=tv.pos; m.pose.position.z=1.0;
    m.pose.orientation.w=1.0;
    m.scale.x=m.scale.y=m.scale.z=2.0;
    m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=0.9f;
    arr.markers.push_back(m);

    visualization_msgs::msg::Marker t;
    t.header.frame_id="map"; t.header.stamp=now();
    t.ns="target_labels"; t.id=(int)k;
    t.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    t.action=visualization_msgs::msg::Marker::ADD;
    t.pose.position=tv.pos; t.pose.position.z=3.5;
    t.pose.orientation.w=1.0;
    t.scale.z=1.5;
    t.color.r=1.0f; t.color.g=1.0f; t.color.b=1.0f; t.color.a=1.0f;
    char buf[64];
    snprintf(buf,sizeof(buf),"#%zu bat:%.0f%%", k+1, tv.remaining_battery);
    t.text=buf;
    arr.markers.push_back(t);
  }
  }  // 结束 if(false)

  visualization_msgs::msg::Marker st;
  st.header.frame_id="map"; st.header.stamp=now();
  st.ns="charging_station"; st.id=0;
  st.type=visualization_msgs::msg::Marker::CUBE;
  st.action=visualization_msgs::msg::Marker::ADD;
  st.pose.position=charging_station_; st.pose.position.z=1.0;
  st.pose.orientation.w=1.0;
  st.scale.x=st.scale.y=st.scale.z=2.5;
  st.color.r=0.0f; st.color.g=0.4f; st.color.b=1.0f; st.color.a=0.9f;
  arr.markers.push_back(st);

  pub_markers_->publish(arr);
}

} // namespace state_machine

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<state_machine::ChargeScheduler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
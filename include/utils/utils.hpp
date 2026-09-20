#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <lanelet2_core/geometry/LineString.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <queue>
#include <nav_msgs/msg/odometry.hpp>
#include <promote_planning_msgs/msg/state_machine.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Geometry>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <promote_planning_msgs/msg/drivable_area.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <lanelet2_core/geometry/LineString.h>


namespace utils
{
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseStamped;
using promote_planning_msgs::msg::StateMachine;
using promote_planning_msgs::msg::DrivableArea;
using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseArray;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using namespace std;
/**
 * @brief 判断点是否在多边形内部
 * @param [in] pose 需要判断的点的pose信息
 * @param [in] polygon 多边形
 */
bool is_point_in_polygon(const Pose& pose, const geometry_msgs::msg::Polygon& polygon);

/**
 * @brief 将当前lanelet中心线转换为vector<point>类型的中心点容器
 * @param [in] centerline lanlet道路中心线
 * @return point容器
 */


/**
 * @brief calculate point start to point end heading
 * @param [in] start_p start point
 * @param [in] end_p end point
 * @return heading
 */
double get_current_heading(const Point& start_p, const Point& end_p); 

/**
 * @brief 找出给定坐标距离pose最近的停车位置索引
 * @param [in] parking_points 所有停车位的中心点
 * @param [in] parking_count 停车位个数
 * @param [in] pose 给定坐标
 * @return 距离给定坐标最近的停车位的索引
 */
int find_nearest_parking_index(std::vector<geometry_msgs::msg::Point> parking_points,
                                size_t parking_count, const Pose& pose);

/**
 * @brief 找到与给定点距离最近的polygons索引
 * @param [in] pose 所给定点
 * @param [in] polygons 所有的polygon
 * @return index
 */
int cal_index(const Pose &pose,const std::vector<geometry_msgs::msg::Polygon> &polygons);

/**
 * @brief 将ConstLineString3d格式转化为std::vector<geometry_msgs::msg::Point>
 * @param [in] line 需要转换的变量
 * @return 转换后的std::vector<geometry_msgs::msg::Point>
 */
std::vector<geometry_msgs::msg::Point> lineToPoints(const lanelet::ConstLineString3d & line);

/**
 * 计算从起点到终点的航向角（yaw）
 * @param start_point 起点
 * @param end_point 终点
 * @return 航向角，单位为弧度，范围在[-π, π]
 */
double calc_yaw(const geometry_msgs::msg::Point & start_point , const geometry_msgs::msg::Point & end_point);

/**
 * @brief 找到与给定点最近的点
 * @param [in] start_point 给定点
 * @param [in] points 点集
 * @return 点集中与给定点最近的点
 */
geometry_msgs::msg::Point findNearestPoint(const geometry_msgs::msg::Point & start_point , 
                                              const std::vector<geometry_msgs::msg::Point> & points);

/**
 * @brief 航向角转换为四元数
 * @return 转换后的四元数
 */
geometry_msgs::msg::Quaternion convertEulerAnglesToQuaternion(double yaw ,double roll = 0.0, double pitch = 0.0);
/**
 * @brief 四元数转换为yaw角
 * @return 航向角
 */
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& orientation);
/**
 * @brief 判断两个角是否在阈值范围内
 * @param [in] yaw1 
 * @param [in] yaw2
 * @param [in] threshold 阈值
 * @return 
 */
bool isSameDirection(double yaw1, double yaw2, double threshold = M_PI / 6.0);

/**
 * @brief point点转换为point32类型并且生成polygon
 * @param [in] points point类型
 * @return polygon
 */
geometry_msgs::msg::Polygon getPolygon(const std::vector<geometry_msgs::msg::Point>& points);

/**
 * @brief 判断两点是否相等
 * @param [in] p1 
 * @param [in] p2
 * @param [in] eps 相等的阈值
 * @return 相等返回true
 */
bool isPointEqual(const geometry_msgs::msg::Point& p1, 
                  const geometry_msgs::msg::Point& p2, 
                  double eps = 1e-6);

/**
 * @brief Point类型是否在polygon内部
 * @param [in] p Point
 * @param [in] points polygon
 * @return if p in polygon return true
 */
bool isInPolygon(const geometry_msgs::msg::Point& p,
                const std::vector<geometry_msgs::msg::Point> & points);

/**
 * @brief 计算两点距离
 */
double calculateDistance(const geometry_msgs::msg::Point &p1,
                             const geometry_msgs::msg::Point &p2);


// bool pointInPolygons(const geometry_msgs::msg::Point& p,
//                      const std::vector<geometry_msgs::msg::Polygon> & polygons);
}
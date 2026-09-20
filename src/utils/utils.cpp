#include "utils/utils.hpp"
namespace utils
{


    bool is_point_in_polygon(const Pose& pose, const geometry_msgs::msg::Polygon& polygon) 
    {
        auto point = pose.position;
        int n = polygon.points.size();
        bool inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if (((polygon.points[i].y > point.y) != (polygon.points[j].y > point.y)) &&
            (point.x < (polygon.points[j].x - polygon.points[i].x) * (point.y - polygon.points[i].y) /
            (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x)) {
            inside = !inside;
            }
        }
        return inside;
    }

    double get_current_heading(const Point& start_p, const Point& end_p)
    {
        const double dx = end_p.x - start_p.x;
        const double dy = end_p.y - start_p.y;
        return std::atan2(dy, dx);
    }

    int find_nearest_parking_index(std::vector<geometry_msgs::msg::Point> parking_points,
                                   size_t parking_count, const Pose& pose) 
    {
        if (parking_count == 0) return -1;

        int nearest_index = -1;
        double min_distance = INFINITY;

        // 获取机器人当前位姿坐标
        const auto& goal_pos = pose.position;

        for (size_t i = 0; i < parking_count; ++i) {
            const auto& point = parking_points[i];
            // 计算三维欧氏距离（若为二维可移除z项）
            double dx = point.x - goal_pos.x;
            double dy = point.y - goal_pos.y;
            double dz = point.z - goal_pos.z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);

            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }
        return nearest_index;
    }

    int cal_index(const Pose &pose,const std::vector<geometry_msgs::msg::Polygon> &polygons)
    {
        if(polygons.size()<1)
        {
            return -1;
        }
        for (size_t i = 0; i < polygons.size(); i++)
        {
            bool is_in = is_point_in_polygon(pose,polygons[i]);
            if(is_in)
            {
                return i;
            }
        }
        return -1;
    }
    std::vector<geometry_msgs::msg::Point> lineToPoints(const lanelet::ConstLineString3d & line)
    {
        std::vector<geometry_msgs::msg::Point> points;
        for (const auto & line_point : line) {
            geometry_msgs::msg::Point point;
            point.x = line_point.basicPoint().x();
            point.y = line_point.basicPoint().y();
            point.z = line_point.basicPoint().z();
            points.push_back(point);
        }
        return points;
    }
    double calc_yaw(const geometry_msgs::msg::Point & start_point , const geometry_msgs::msg::Point & end_point)
    {
        double dx = end_point.x - start_point.x;
        double dy = end_point.y - start_point.y;
        if(dx == 0.0 && dy == 0.0)
        {
            return 0.0;
        }
        double yaw = std::atan2(dy,dx);
        while(yaw < 0.0){
            yaw += 2*M_PI;
        }
        while(yaw >= 2*M_PI){
            yaw -= 2*M_PI;
        }
        return yaw;

    }

    geometry_msgs::msg::Point findNearestPoint(const geometry_msgs::msg::Point & start_point , 
                                               const std::vector<geometry_msgs::msg::Point> & points){
        geometry_msgs::msg::Point point;
        double min_distance = 1e8;
        for(const auto & p : points)
        {
            double distance = std::sqrt(std::pow(p.x - start_point.x,2)+std::pow(p.y - start_point.y,2));
           if(distance < min_distance)
           {
                min_distance = distance;
                point = p;
           }
        }
        return point;
    }

    geometry_msgs::msg::Quaternion convertEulerAnglesToQuaternion(double yaw ,double roll, double pitch)
    {
        geometry_msgs::msg::Quaternion quaternion;
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        // Calculate the elements of the quaternion
        quaternion.w = cr * cp * cy + sr * sp * sy;
        quaternion.x = sr * cp * cy - cr * sp * sy;
        quaternion.y = cr * sp * cy + sr * cp * sy;
        quaternion.z = cr * cp * sy - sr * sp * cy;
        return quaternion;
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& orientation) {
        // 使用四元数初始化 tf2::Quaternion
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        
        // 将四元数转换为旋转矩阵
        tf2::Matrix3x3 m(q);
        
        // 获取欧拉角，rpy.x = roll, rpy.y = pitch, rpy.z = yaw
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 返回偏航角 (yaw)
        return yaw;
    }

    bool isSameDirection(double yaw1, double yaw2, double threshold) 
    {
        // 计算两个角度的差
        double diff = yaw1 - yaw2;
        
        // 处理角度差的周期性，将其归一化到[-π, π]范围
        diff = fmod(diff, 2 * M_PI);
        if (diff > M_PI) {
            diff -= 2 * M_PI;
        } else if (diff < -M_PI) {
            diff += 2 * M_PI;
        }
        
        // 取角度差的绝对值
        double abs_diff = fabs(diff);
        
        // 如果角度差小于阈值，或者大于2π-阈值（即几乎相反方向的补角）
        // 则认为它们是同向的
        return abs_diff < threshold || abs_diff > 2 * M_PI - threshold;
    }

    double calculateDistance(const geometry_msgs::msg::Point &p1,
                             const geometry_msgs::msg::Point &p2)
    {
        return std::sqrt(std::pow(p1.x-p2.x,2)+std::pow(p1.y-p2.y,2));
    }

    geometry_msgs::msg::Polygon getPolygon(const std::vector<geometry_msgs::msg::Point>& points)
    {
        geometry_msgs::msg::Polygon polygon;
        for(const auto p:points)
        {
            geometry_msgs::msg::Point32 new_p;
            new_p.x = p.x;
            new_p.y = p.y;
            new_p.z = p.z;
            polygon.points.push_back(new_p);
        }
        return polygon;
    }
    bool isInPolygon(const geometry_msgs::msg::Point& p,
                     const std::vector<geometry_msgs::msg::Point> & points)
    {
        for(const auto & point : points)
        {
            if(isPointEqual(point,p))
            {
                return true;
            }
        }
        return false;
    }
    bool isPointEqual(const geometry_msgs::msg::Point& p1, 
                  const geometry_msgs::msg::Point& p2, 
                  double eps)
    {
        return std::fabs(p1.x - p2.x) < eps &&
            std::fabs(p1.y - p2.y) < eps &&
            std::fabs(p1.z - p2.z) < eps;
    }
    // bool pointInPolygons(const geometry_msgs::msg::Point& p,
    //                      const std::vector<geometry_msgs::msg::Polygon> & polygons)
    // {
    //     for(const auto polygon : polygons)
    //     {
    //         if(is_point_in_polygon(p,polygon))
    //             return true;
    //     }
    //     return false;
    // }

}
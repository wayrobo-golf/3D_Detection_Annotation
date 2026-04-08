/**
 * @file common_utils.h
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SUPPORT_COMMON_UTILS_INCLUDE_COMMON_UTILS_COMMON_UTILS_H_
#define SUPPORT_COMMON_UTILS_INCLUDE_COMMON_UTILS_COMMON_UTILS_H_

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

namespace common_utils {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-6;
constexpr uint32_t kHeartbeatInterval = 10;  // s
constexpr auto kGetParamNodeRadomLen = 6;
constexpr auto kBufLen_16 = 16;
constexpr auto kDefaultTtl = 30;
constexpr auto kWeekDayCnt = 7;
constexpr auto kLeapYearInterval = 4;
constexpr auto kCnstInt10 = 10;
constexpr auto kCnstInt26 = 26;
constexpr auto kCnstInt100 = 100;
constexpr auto kCenturyYearCnt = 100;
constexpr auto kYearMonthCnt = 12;
constexpr auto kTmStartYear = 1900;
constexpr auto kDegreeOfPi = 180.0;
constexpr auto kTimeStrBufLen = 32;
constexpr auto kTimeUnitRatio = 1000;

enum class TimeUnitType { kSeconds = 0, kMilliseconds };

class NonCopyable {
 public:
  NonCopyable(NonCopyable const &) = delete;
  NonCopyable &operator=(NonCopyable const &) = delete;

 protected:
  NonCopyable() = default;
  ~NonCopyable() = default;

  NonCopyable(NonCopyable &&) = default;
  NonCopyable &operator=(NonCopyable &&) = default;
};

class CommonUtils : NonCopyable {
 public:
  /**
   * @brief 将字符串转换为小写形式
   *
   * @param str 需要转换的字符串
   * @return string 转换后的字符串, 错误时返回空字符串
   */
  static std::string ToLower(const std::string &str);

  /**
   * @brief 生成当前时间的毫秒形式的unix时间戳
   *
   * @return uint64_t unix时间戳
   */
  static uint64_t GetCurrentTimestamp(
      TimeUnitType unit_type = TimeUnitType::kMilliseconds) {
    if (unit_type == TimeUnitType::kMilliseconds) {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch())
          .count();
    }

    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static std::string GetCurrentStrTime(bool split = true) {
    // 将时间戳转换为 tm 结构
    uint64_t timestamp = GetCurrentTimestamp(TimeUnitType::kMilliseconds);
    auto time_sec = static_cast<time_t>(timestamp / kTimeUnitRatio);
    struct tm tm_info{};
    localtime_r(&time_sec, &tm_info);

    // 格式化时间字符串
    std::array<char, kTimeStrBufLen> buffer{};

    if (split) {
      strftime(buffer.data(), buffer.size(), "%Y-%m-%d %H:%M:%S", &tm_info);
      std::string fill_string = timestamp % kTimeUnitRatio < kCnstInt10    ? "00"
                                : timestamp % kTimeUnitRatio < kCnstInt100 ? "0"
                                                         : "";
      return std::string(buffer.data()) + "." + fill_string +
             std::to_string(timestamp % kTimeUnitRatio);
    }

    strftime(buffer.data(), buffer.size(), "%Y%m%d%H%M%S", &tm_info);
    return buffer.data();
  }

  /**
   * @brief 生成长度为`str_len`的随机字符串
   *
   * @param str_len 指定随机字符串的长度
   * @return std::string 随机字符串长度
   */
  static std::string GenerateRandomStr(size_t str_len);

  /**
   * @brief 度转弧度
   *
   * @param degree 度
   * @return double 弧度
   */
  static double Degree2Radian(double degree) { return degree * kPi / kDegreeOfPi; }

  /**
   * @brief 弧度转度
   *
   * @param radian 弧度
   * @return double 度
   */
  static double Radian2Degree(double radian) { return radian * kDegreeOfPi / kPi; }

  static int GetWeekDay() {
    time_t now = time(nullptr);
    struct tm cur_t{};
    localtime_r(&now, &cur_t);

    int day_in_month = cur_t.tm_mday;
    int month = cur_t.tm_mon + 1;
    int year = cur_t.tm_year + kTmStartYear;

    int tmp_y = 0;
    int tmp_c = 0;
    int tmp_m = 0;
    int tmp_d = 0;

    if (month == 1 || month == 2) {
      tmp_c = (year - 1) / kCenturyYearCnt;
      tmp_y = (year - 1) % kCenturyYearCnt;
      tmp_m = month + kYearMonthCnt;
      tmp_d = day_in_month;
    } else {
      tmp_c = year / kCenturyYearCnt;
      tmp_y = year % kCenturyYearCnt;
      tmp_m = month;
      tmp_d = day_in_month;
    }
    // 蔡勒公式
    int week_day = tmp_y + tmp_y / kLeapYearInterval + tmp_c / kLeapYearInterval - 2 * tmp_c +
                   kCnstInt26 * (tmp_m + 1) / kCnstInt10 + tmp_d - 1;
    // iWeek为负时取模
    week_day = week_day >= 0 ? (week_day % kWeekDayCnt) : (week_day % kWeekDayCnt + kWeekDayCnt);
    if (week_day == 0) {
      week_day = kWeekDayCnt;
    }

    return week_day - 1;
  }

  template <typename T>
  static bool IsPntInsidePolygon(const geometry_msgs::msg::Polygon &polygon,
                                 const T &pnt) {
    if (polygon.points.empty()) {
      return false;
    }

    auto num_vertices = polygon.points.size();
    bool inside = false;

    // Store the first point in the polygon and initialize
    // the second point
    geometry_msgs::msg::Point32 point1 = polygon.points[0];
    geometry_msgs::msg::Point32 point2;

    // Loop through each edge in the polygon
    for (size_t i = 1; i <= num_vertices; i++) {
      // Get the next point in the polygon
      point2 = polygon.points[i % num_vertices];

      // Check if the point is above the minimum y
      // coordinate of the edge
      if (pnt.y > std::min(point1.y, point2.y)) {
        // Check if the point is below the maximum y
        // coordinate of the edge
        if (pnt.y <= std::max(point1.y, point2.y)) {
          // Check if the point is to the left of the
          // maximum x coordinate of the edge
          if (pnt.x <= std::max(point1.x, point2.x)) {
            // Calculate the x-intersection of the
            // line connecting the point to the edge
            double x_intersection =
                (pnt.y - point1.y) * (point2.x - point1.x) / (point2.y - point1.y) + point1.x;

            // Check if the point is on the same
            // line as the edge or to the left of
            // the x-intersection
            if (point1.x == point2.x || pnt.x <= x_intersection) {
              // Flip the inside flag
              inside = !inside;
            }
          }
        }
      }

      // Store the current point as the first point for
      // the next iteration
      point1 = point2;
    }

    // Return the value of the inside flag
    return inside;
  }

  template <typename T>
  static geometry_msgs::msg::Point32 GetDistPnt(const T &pnt, double distance,
                                                double angle) {
    geometry_msgs::msg::Point32 res;

    res.x =
        pnt.x + distance * cos(common_utils::CommonUtils::Degree2Radian(angle));
    res.y =
        pnt.y + distance * sin(common_utils::CommonUtils::Degree2Radian(angle));

    return res;
  }

  static geometry_msgs::msg::Quaternion CalRosQuanFromYaw(double yaw);
  static double CalYawFromRosQuan(const geometry_msgs::msg::Quaternion &quat);
  static geometry_msgs::msg::Quaternion YawToQuaternion(double yaw);
  static double CalInsYawRadFromRosQuan(const geometry_msgs::msg::Quaternion& ros_quat_msg);

  static geometry_msgs::msg::Polygon CalcOuterPolygonFromPose(
      const geometry_msgs::msg::Pose &pose, double width, double height,
      double scale);

  static float DistanceToPolygon(const geometry_msgs::msg::Point32 &point,
                                 const geometry_msgs::msg::Polygon &polygon);

  static std::string ExecuteCmd(const char *cmd);
  static std::string ExecuteCmd(const std::string &cmd);

  static bool RotateFile(const std::string &dir_path,
                         const std::string &file_name);

  template <typename T>
  static bool IsParamInRange(const T &param, const T &lower, const T &upper) {
    return param >= lower && param <= upper;
  }
  static double calculateDistance(const geometry_msgs::msg::Point32 &pnt1,
                                  const geometry_msgs::msg::Point32 &pnt2) {
    return sqrt(pow(pnt1.x - pnt2.x, 2) + pow(pnt1.y - pnt2.y, 2));
  }
  static double calculateDistance(const geometry_msgs::msg::Point &pnt1,
                                  const geometry_msgs::msg::Point &pnt2) {
    return sqrt(pow(pnt1.x - pnt2.x, 2) + pow(pnt1.y - pnt2.y, 2));
  }
  // 计算两个点之间的方向角
  static double calculateYaw(const geometry_msgs::msg::Point32 &current_point,
                             const geometry_msgs::msg::Point32 &next_point) {
    double delta_x = next_point.x - current_point.x;
    double delta_y = next_point.y - current_point.y;
    return atan2(delta_y, delta_x);
  }

  static double calculateYaw(const geometry_msgs::msg::Point &current_point,
                             const geometry_msgs::msg::Point &next_point) {
    double delta_x = next_point.x - current_point.x;
    double delta_y = next_point.y - current_point.y;
    return atan2(delta_y, delta_x);
  }

  static bool waitForTimeSync(std::shared_ptr<rclcpp::Node> node_handle) {
    auto get_time_sync_cli = node_handle->create_client<std_srvs::srv::Trigger>(
        "/time_manager/is_synchronized");
    return get_time_sync_cli->wait_for_service();
  }

  static double calculateYawError(double yaw_1, double yaw_2) {
    double ang_dist = fabs(yaw_1 - yaw_2);
    while (ang_dist > M_PI) {
      ang_dist -= 2 * M_PI;
      ang_dist = fabs(ang_dist);
    }
    return ang_dist;
  }

  template <typename T>
  static bool GetParam(rclcpp::Node::SharedPtr node_h, const std::string &node_name,
                       const std::string &para_name, T &para) {
    if (node_name.empty() && para_name.empty()) {
      return false;
    }

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
        std::string(node_h->get_name()) + "_get_param_" + GenerateRandomStr(kGetParamNodeRadomLen));
    auto para_cli =
        std::make_shared<rclcpp::SyncParametersClient>(node, node_name);

    if (!para_cli->wait_for_service(std::chrono::seconds(3))) {
      std::cerr << "service_is_ready is false,node (" << node_name << ") get param:" << para_name << std::endl;
      return false;
    }

    if (para_cli->has_parameter(para_name)) {
      para = para_cli->get_parameter<T>(para_name);
      return true;
    }

    std::cerr << "Parameter " << para_name << " not found in target node." << std::endl;

    return false;
  }

  template <typename T>
  static void GetParam(rclcpp_lifecycle::LifecycleNode::SharedPtr node_h,
                       const std::string &node_name,
                       const std::string &para_name, T &para) {
    if (node_name.empty() && para_name.empty()) {
      return;
    }

    auto para_cli =
        std::make_shared<rclcpp::SyncParametersClient>(node_h, node_name);

    if (!para_cli->wait_for_service(std::chrono::seconds(3))) {
      std::cerr << "service_is_ready is false,node (" << node_name << ") get param:" << para_name << std::endl;
      return;
    }

    if (para_cli->has_parameter(para_name)) {
      para = para_cli->get_parameter<T>(para_name);
    } else {
      std::cerr << "Parameter " << para_name << " not found in target node." << std::endl;
    }
  }

  template <typename T>
  static bool SetParam(rclcpp::Node::SharedPtr node_h, const std::string &node_name,
                       const std::string &param_name, T para) {
    if (node_name.empty() && param_name.empty()) {
      return false;
    }

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
        std::string(node_h->get_name()) + "_set_param");
    auto para_cli =
        std::make_shared<rclcpp::SyncParametersClient>(node, node_name);

    if (!para_cli->wait_for_service(std::chrono::seconds(3))) {
      std::cerr << "service_is_ready is false,node (" << node_name << ") set param:" << param_name << std::endl;
      return false;
    }

    auto results = para_cli->set_parameters({rclcpp::Parameter(param_name, para)});

    return std::any_of(results.begin(), results.end(),
      [](const rcl_interfaces::msg::SetParametersResult& result) { return result.successful; });
  }

  /**
   * @brief 尝试 ping 一次指定的 ip
   *
   * @param ip 指定的 ip 地址
   * @return true ping 返回成功
   * @return false ping 返回失败
   */
  static bool Ping(const std::string &ip_str, const std::string &inface = "") {
    std::string command;
    if (!inface.empty()) {
      command = "ping -I " + inface + " -c 1 -w 1 " + ip_str + " > /dev/null 2>&1";
    } else {
      command = "ping -c 1 -w 1 " + ip_str + " > /dev/null 2>&1";
    }
    int ret = std::system(command.c_str());
    return ret == 0;
  }

  static geometry_msgs::msg::Pose ReversePose(geometry_msgs::msg::Pose pose) {
    auto yaw = CalYawFromRosQuan(pose.orientation);
    pose.orientation = YawToQuaternion(yaw + M_PI);
    return pose;
  }

  static bool FindMindistDirection(
      geometry_msgs::msg::Pose current_pose,
      std::vector<geometry_msgs::msg::Pose> pnt_list) {
    double min_dist = DBL_MAX;
    geometry_msgs::msg::Pose min_dist_pnt;
    for (auto &pnt : pnt_list) {
      auto dist = sqrt(pow(pnt.position.x - current_pose.position.x, 2) +
                       pow(pnt.position.y - current_pose.position.y, 2));
      if (dist < min_dist) {
        min_dist = dist;
        min_dist_pnt = pnt;
      }
    }

    auto yaw = CalYawFromRosQuan(current_pose.orientation);
    auto correct_yaw = CalYawFromRosQuan(min_dist_pnt.orientation);
    auto yaw_error = calculateYawError(yaw, correct_yaw);
    return fabs(yaw_error) < M_PI / 2;
  }

  static size_t FindMindistIndex(geometry_msgs::msg::Pose current_pose,
                        std::vector<geometry_msgs::msg::Pose> pnt_list) {
  double min_dist = DBL_MAX;
  size_t res = 0;
  for (size_t i = 0; i < pnt_list.size(); i++) {
    auto &pnt = pnt_list[i];
    auto dist = sqrt(pow(pnt.position.x - current_pose.position.x, 2) +
                      pow(pnt.position.y - current_pose.position.y, 2));
    if (dist < min_dist) {
      min_dist = dist;
      res = i;
    }
  }
  return res;
}

  static bool CheckDirection(geometry_msgs::msg::Pose in_pose,
                             geometry_msgs::msg::Pose out_pose) {
    double in_angle = CalYawFromRosQuan(in_pose.orientation);
    double dest_angle = atan2(out_pose.position.y - in_pose.position.y,
                              out_pose.position.x - in_pose.position.x);
    return CheckDirection(dest_angle, in_angle);
  }

  static bool CheckDirection(double angle1, double angle2) {
    double angle_error = angle1 - angle2;
    if (angle_error > kPi) {
      angle_error = angle_error - 2 * kPi;
    } else if (angle_error < -kPi) {
      angle_error = angle_error + 2 * kPi;
    }

    return fabs(angle_error) < common_utils::kPi / 2;
  }

  static bool CheckFileExist(const std::string &file_path) {
    return ::access(file_path.c_str(), F_OK) != -1;
  }

  static double GetMindist(geometry_msgs::msg::Pose current_pose,
                           std::vector<geometry_msgs::msg::Pose> pnt_list) {
    double min_dist = DBL_MAX;
    for (auto &pnt : pnt_list) {
      auto dist = sqrt(pow(pnt.position.x - current_pose.position.x, 2) +
                       pow(pnt.position.y - current_pose.position.y, 2));
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
    return min_dist;
  }

  static std::string GetCurrentYearWeek() {
    std::time_t t_stamp = std::time(nullptr);
    std::tm tm_info = *std::localtime(&t_stamp);
    std::array<char, kBufLen_16> buf{};
    std::strftime(buf.data(), buf.size(), "%Y-%W", &tm_info);
    return buf.data();
  }

  static std::vector<std::string> StrSplit(const std::string &str, char delim);

  static bool RobotParamsSaveToRedis(const std::string& param_str, const std::string& channel = "robot_params");
  static void SubscribeRedisData(const std::function<void(std::string, std::string)>& call_back,
    const std::string& channel = "robot_params");
  static void UnSubscribRedis();
  static bool WriterRedis(const std::string& key, const std::string& value,
                          const std::chrono::seconds& ttl = std::chrono::seconds(kDefaultTtl));
  static std::string RedisGetValue(const std::string& key);

  static std::string GetExecutableFileName();
  static pid_t GetPidByExeFile(const std::string& exe_file);

 private:
  static float DistanceToSegment(const geometry_msgs::msg::Point32 &pnt_p,
                                 const geometry_msgs::msg::Point32 &pnt_a,
                                 const geometry_msgs::msg::Point32 &pnt_b);

  CommonUtils() = default;
  ~CommonUtils() = default;
};

}  // namespace common_utils

#endif  // SUPPORT_COMMON_UTILS_INCLUDE_COMMON_UTILS_COMMON_UTILS_H_

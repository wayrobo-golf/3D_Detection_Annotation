/**
 * @file common_utils.cc
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "common_utils/common_utils.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <mutex>
#include <thread>
#include <filesystem>
#include <iostream>
#include <vector>
#include <memory>

#include <boost/core/no_exceptions_support.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <sw/redis++/redis++.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

using std::string;
using std::vector;

namespace {
constexpr int64_t kMaxLogFileSize = static_cast<int64_t>(100) * 1024 * 1024;   // 100 MB
constexpr auto kCmdRetBufLen = 2048;
constexpr auto kDegree_90 = 90.0;
constexpr auto kBuffLen_1024 = 1024;
constexpr auto kRedisSubInterval = 10;  // ms
constexpr auto kRedisInterval_100ms = 100;

// 计算两个向量的点积
float dot(const geometry_msgs::msg::Point32 &vec1, const geometry_msgs::msg::Point32 &vec2) {
  return vec1.x * vec2.x + vec1.y * vec2.y;
}

// 计算向量的差
geometry_msgs::msg::Point32 operator-(const geometry_msgs::msg::Point32 &pnt1,
                                 const geometry_msgs::msg::Point32 &pnt2) {
  geometry_msgs::msg::Point32 result;
  result.x = pnt1.x - pnt2.x;
  result.y = pnt1.y - pnt2.y;
  return result;
}

geometry_msgs::msg::Point32 operator+(const geometry_msgs::msg::Point32 &pnt1,
                                 const geometry_msgs::msg::Point32 &pnt2) {
  geometry_msgs::msg::Point32 result;
  result.x = pnt1.x + pnt2.x;
  result.y = pnt1.y + pnt2.y;
  return result;
}

// 计算向量的模
float norm(const geometry_msgs::msg::Point32 &vec) {
  return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::mutex g_redis_mutex;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
bool g_redis_start_sub = false;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::string g_redis_subscribe_channel;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::unique_ptr<sw::redis::Redis> g_redis_cli = nullptr;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::unique_ptr<std::thread> g_redis_sub_thread = nullptr;
}  // namespace

namespace common_utils {

string CommonUtils::ToLower(const string &str) {
  string res;
  res.reserve(str.size());

  for (const auto &character : str) {
    if (isalpha(character) == 0) {
      return "";
    }

    res.push_back(static_cast<char>(tolower(character)));
  }

  return res;
}

string CommonUtils::GenerateRandomStr(size_t str_len) {
  string character_set =
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

  string random_str;
  std::random_device r_dev;
  std::mt19937 gen(r_dev());
  std::uniform_int_distribution<> dis(0, static_cast<int>(character_set.size()) - 1);

  for (size_t i = 0; i < str_len; ++i) {
    random_str.push_back(character_set[dis(gen)]);
  }

  return random_str;
}

geometry_msgs::msg::Quaternion CommonUtils::CalRosQuanFromYaw(double yaw) {
  static tf2::Quaternion Z90_DEG_ROTATION;
  static bool is_init = false;
  if (!is_init) {
    Z90_DEG_ROTATION.setRPY(0, 0,
                            common_utils::CommonUtils::Degree2Radian(kDegree_90));
    is_init = true;
  }

  tf2::Quaternion enu_orientation;
  enu_orientation.setRPY(0, 0, -common_utils::CommonUtils::Degree2Radian(yaw));
  tf2::Quaternion ros_orientation = Z90_DEG_ROTATION * enu_orientation;

  return tf2::toMsg(ros_orientation);
}

double CommonUtils::CalYawFromRosQuan(const geometry_msgs::msg::Quaternion &quat) {
  tf2::Quaternion tf2_q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 matrx(tf2_q);
  double roll;
  double pitch;
  double yaw;
  matrx.getRPY(roll, pitch, yaw);

  return yaw;
}

double CommonUtils::CalInsYawRadFromRosQuan(const geometry_msgs::msg::Quaternion& ros_quat_msg) {
  static tf2::Quaternion Z90_DEG_ROTATION;
  static bool is_init = false;
  if (!is_init) {
    Z90_DEG_ROTATION.setRPY(0, 0, common_utils::CommonUtils::Degree2Radian(kDegree_90));
    is_init = true;
  }

  tf2::Quaternion ros_orientation;
  tf2::fromMsg(ros_quat_msg, ros_orientation);

  tf2::Quaternion enu_orientation = Z90_DEG_ROTATION.inverse() * ros_orientation;

  double roll;
  double pitch;
  double yaw_rad;
  tf2::Matrix3x3(enu_orientation).getRPY(roll, pitch, yaw_rad);

  return -yaw_rad;
}

geometry_msgs::msg::Quaternion CommonUtils::YawToQuaternion(double yaw) {
    // 创建 tf2 的四元数
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(0, 0, yaw);  // 仅设置绕 Z 轴的旋转 (Yaw)

    // 转换为 geometry_msgs::Quaternion
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = tf2_quaternion.x();
    quaternion.y = tf2_quaternion.y();
    quaternion.z = tf2_quaternion.z();
    quaternion.w = tf2_quaternion.w();

    return quaternion;
}

geometry_msgs::msg::Polygon CommonUtils::CalcOuterPolygonFromPose(
    const geometry_msgs::msg::Pose &pose, double width, double height,
    double scale) {
  geometry_msgs::msg::Polygon polygon;

  // 使用旋转矩阵将宽和高绕Z轴旋转
  double yaw = tf2::getYaw(pose.orientation);
  double cos_theta = cos(yaw);
  double sin_theta = sin(yaw);

  // 四个顶点相对于多边形中心的坐标
  geometry_msgs::msg::Point32 point1;
  geometry_msgs::msg::Point32 point2;
  geometry_msgs::msg::Point32 point3;
  geometry_msgs::msg::Point32 point4;
  point1.x = static_cast<float>(sin_theta * (-width / 2) + cos_theta * (-height * scale) + pose.position.x);
  point1.y = static_cast<float>(cos_theta * (width / 2) + sin_theta * (-height * scale) + pose.position.y);

  point2.x = static_cast<float>(sin_theta * (-width / 2) + cos_theta * (height * (1.0 - scale)) + pose.position.x);
  point2.y = static_cast<float>(cos_theta * (width / 2) + sin_theta * (height * (1.0 - scale)) + pose.position.y);

  point3.x = static_cast<float>(sin_theta * (width / 2) + cos_theta * (height * (1.0 - scale)) + pose.position.x);
  point3.y = static_cast<float>(cos_theta * (-width / 2) + sin_theta * (height * (1.0 - scale)) + pose.position.y);

  point4.x = static_cast<float>(sin_theta * (width / 2) + cos_theta * (-height * scale) + pose.position.x);
  point4.y = static_cast<float>(cos_theta * (-width / 2) + sin_theta * (-height * scale) + pose.position.y);

  // 将四个点添加到多边形
  polygon.points.push_back(point1);
  polygon.points.push_back(point2);
  polygon.points.push_back(point3);
  polygon.points.push_back(point4);
  polygon.points.push_back(point1);

  return polygon;
}

float CommonUtils::DistanceToPolygon(const geometry_msgs::msg::Point32 &point,
                                     const geometry_msgs::msg::Polygon &polygon) {
  float min_dist = FLT_MAX;
  if (polygon.points.empty()) {
    return min_dist;
  }

  for (size_t i = 0; i < polygon.points.size(); ++i) {
    // 计算多边形的边表示为向量
    geometry_msgs::msg::Point32 edge_start = polygon.points[i];
    geometry_msgs::msg::Point32 edge_end =
        polygon.points[(i + 1) % polygon.points.size()];

    float distance = DistanceToSegment(point, edge_start, edge_end);

    // 更新最小距离
    min_dist = std::min(min_dist, distance);
  }

  return min_dist;
}

string CommonUtils::ExecuteCmd(const char *cmd) {
  std::array<char, kBuffLen_1024> buf{};
  FILE *fptr = popen(cmd, "r");

  if (fptr == nullptr) {
    return "";
  }

  string res;
  while (fgets(buf.data(), buf.size(), fptr) != nullptr) {
    res += buf.data();
  }

  pclose(fptr);

  if (!res.empty() && res[res.size() - 1] == '\n') {
    res.erase(res.size() - 1);
  }

  return res;
}

string CommonUtils::ExecuteCmd(const std::string &cmd) {
  return ExecuteCmd(cmd.c_str());
}

bool CommonUtils::RotateFile(const string &dir_path, const string &file_name) {
  std::string file_path = dir_path + file_name;
  std::ifstream rfile(file_path);
  if (!rfile.is_open()) {
    return false;
  }

  rfile.seekg(0, std::ifstream::end);
  auto file_size = rfile.tellg();
  rfile.close();

  if (file_size > kMaxLogFileSize) {
    time_t t_stamp = time(nullptr);
    string new_file_path = dir_path + std::to_string(t_stamp) + "_" + file_name;
    ::rename(file_path.c_str(), new_file_path.c_str());
  }

  return true;
}

float CommonUtils::DistanceToSegment(const geometry_msgs::msg::Point32 &pnt_p,
                                     const geometry_msgs::msg::Point32 &pnt_a,
                                     const geometry_msgs::msg::Point32 &pnt_b) {
  // 计算向量 AB
  geometry_msgs::msg::Point32 seg_ab = pnt_b - pnt_a;

  // 计算向量 AP
  geometry_msgs::msg::Point32 seg_ap = pnt_p - pnt_a;

  // 计算投影长度
  float prj_len = dot(seg_ap, seg_ab) / dot(seg_ab, seg_ab);

  // 如果 prj_len 在 [0, 1] 内，表示投影在边 AB 上
  if (prj_len >= 0.0 && prj_len <= 1.0) {
    // 计算投影点 Proj
    seg_ab.x = prj_len * seg_ab.x;
    seg_ab.y = prj_len * seg_ab.y;
    seg_ab.z = prj_len * seg_ab.z;
    geometry_msgs::msg::Point32 proj = pnt_a + seg_ab;

    // 计算点到投影点的距离
    return norm(pnt_p - proj);
  }
  // 如果 prj_len < 0，点在边 AB 之前；如果 prj_len > 1，点在边 AB 之后
  // 返回点 P 到边 AB 端点 A 和 B 的最小距离
  return std::min(norm(pnt_p - pnt_a), norm(pnt_p - pnt_b));
}

std::vector<std::string> CommonUtils::StrSplit(const std::string& str, char delim) {
  std::vector<std::string> retVec;
  std::istringstream iss(str);
  std::string token;

  while (std::getline(iss, token, delim)) {
    if (token.empty()) {
      continue;
    }
    retVec.push_back(token);
  }

  return retVec;
}

static bool CheckRedisUrlValid(const std::string& url) {
  try {
    sw::redis::Redis redis(url + "?socket_timeout=500ms&connect_timeout=500ms");
    std::string response = redis.ping();
    return (response == "PONG");
  } catch (const std::exception&  except) {
    std::cerr << "redis check exception" << except.what() << std::endl;
  }

  return false;
}

static std::string GetReidsUrl() {
  std::string url;
  std::string yaml_file_path =
    ament_index_cpp::get_package_share_directory("common_utils") + "/config/common_utils_conf.yaml";
  if (std::filesystem::exists(yaml_file_path)) {
    try {
      YAML::Node config = YAML::LoadFile(yaml_file_path);
      auto ip_str = config["redis_ip"].as<std::string>();
      auto port = config["redis_port"].as<std::string>();
      url = "tcp://" + ip_str + ":" + port;
    } catch (const std::exception&  except) {
       std::cerr << "common_utils_conf.yaml read exception" << except.what() << std::endl;
    }
  }

  if (url.empty()) {
    url = "tcp://127.0.0.1:6379";
    if (!CheckRedisUrlValid(url)) {
      url = "tcp://172.31.31.4:6379";
    }
  }

  std::cout<< url << std::endl;

  return url;
}

bool CommonUtils::RobotParamsSaveToRedis(const std::string& param_str, const std::string& channel) {
  try {
    {
      std::lock_guard<std::mutex> locker(g_redis_mutex);
      if (!g_redis_cli) {
        g_redis_cli = std::make_unique<sw::redis::Redis>(
                        GetReidsUrl() + "?keep_alive=true&pool_size=3&connect_timeout=500ms");
      }
    }

    g_redis_cli->publish(channel, param_str);
    return true;
  } catch (const std::exception &e) {
    std::cout << "exception:" << e.what() << std::endl;
    return false;
  }
}

void RedisSubThreadFunc(const std::function<void(std::string, std::string)>& call_back) {
  try {
    sw::redis::Redis redis_sub(GetReidsUrl() + "?connect_timeout=500ms");
    auto sub_scriber = redis_sub.subscriber();
    sub_scriber.on_message(call_back);
    sub_scriber.subscribe(g_redis_subscribe_channel);
    g_redis_start_sub = true;

    while (g_redis_start_sub) {
      sub_scriber.consume();
      std::this_thread::sleep_for(std::chrono::milliseconds(kRedisSubInterval));
    }

    sub_scriber.unsubscribe();
    sub_scriber.consume();
  } catch (const std::exception &e) {
    std::cout << "exception:" << e.what() << std::endl;
  }
}

void CommonUtils::SubscribeRedisData(const std::function<void(std::string, std::string)>& call_back,
  const std::string& channel) {
  try {
    {
      std::lock_guard<std::mutex> locker(g_redis_mutex);
      if (!g_redis_sub_thread) {
        g_redis_subscribe_channel = channel;
        g_redis_sub_thread = std::make_unique<std::thread>(RedisSubThreadFunc, call_back);
        std::this_thread::sleep_for(std::chrono::milliseconds(kRedisInterval_100ms));
      } else {
        std::cout << "subscribe exist!" << std::endl;
        return;
      }
    }
  } catch (const std::exception &e) {
    std::cout << "exception:" << e.what() << std::endl;
  }
}

void CommonUtils::UnSubscribRedis() {
  try {
    {
      std::lock_guard<std::mutex> locker(g_redis_mutex);
      if (!g_redis_sub_thread) {
        std::cout << "subscribe not exist!" << std::endl;
        return;
      }
    }
    g_redis_start_sub = false;
    RobotParamsSaveToRedis("{}", g_redis_subscribe_channel);  // 避免consume死等
    if (g_redis_sub_thread->joinable()) {
      g_redis_sub_thread->join();
    }
    g_redis_sub_thread.reset();
    g_redis_subscribe_channel.clear();
  } catch (const std::exception &e) {
    std::cout << "exception:" << e.what() << std::endl;
  }
}

bool CommonUtils::WriterRedis(const std::string& key, const std::string& value,
                              const std::chrono::seconds& ttl) {
  try {
    std::lock_guard<std::mutex> locker(g_redis_mutex);
    if (!g_redis_cli) {
      g_redis_cli = std::make_unique<sw::redis::Redis>(
                      GetReidsUrl() + "?keep_alive=true&pool_size=3&connect_timeout=500ms");
    }

    return g_redis_cli->set(key, value, ttl);
  } catch (const std::exception &e) {
    std::cout << "exception:" << e.what() << std::endl;
    return false;
  }
}

std::string CommonUtils::RedisGetValue(const std::string& key) {
  try {
    std::lock_guard<std::mutex> locker(g_redis_mutex);
    if (!g_redis_cli) {
      g_redis_cli = std::make_unique<sw::redis::Redis>(
                      GetReidsUrl() + "?keep_alive=true&pool_size=3&connect_timeout=500ms");
    }

    auto value = g_redis_cli->get(key);
    if (value) {
      return *value;
    }
    return "";
  } catch (const std::exception &e) {
    std::cout << "exception:" << e.what() << std::endl;
    return "";
  }
}

std::string CommonUtils::GetExecutableFileName() {
  static std::string exe_file;
  if (exe_file.empty()) {
    std::array<char, PATH_MAX> file_path{};
    ssize_t len = readlink("/proc/self/exe", file_path.data(), PATH_MAX - 1);
    if (len <= 0) {
        return "";
    }

    file_path.at(len) = '\0';
    std::string path_str(file_path.data());
    size_t pos = path_str.find_last_of('/');
    exe_file = pos == std::string::npos ? path_str : path_str.substr(pos + 1);
  }
  return exe_file;
}

pid_t CommonUtils::GetPidByExeFile(const std::string& exe_file) {
  pid_t pid = -1;
  if (exe_file.empty()) {
    return pid;
  }

  std::string cmd = "ps -eo pid,cmd 2>/dev/null | grep -F " + exe_file;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    std::cerr << "执行命令失败：" << strerror(errno) << std::endl;
    return pid;
  }

  std::array<char, kCmdRetBufLen> buf{};
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) {
    auto token = StrSplit(buf.data(), ' ');
    if (token.size() <= 1) {
      continue;
    }

    std::string cmd_path = token[1];
    if (cmd_path == "python3") {
      if (token.size() <= 2) {
        continue;
      }
      cmd_path = token[2];
    }
    size_t pos = cmd_path.find_last_of('/');
    if (pos == std::string::npos || cmd_path.substr(pos + 1) != exe_file) {
      continue;
    }

    pid = static_cast<pid_t>(atoi(token[0].c_str()));
    if (pid > 0) {
      break;
    }
  }

  pclose(pipe);
  return pid;
}

}  // namespace common_utils

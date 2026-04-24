/**
 * @file logger.h
 * @author zjs
 * @brief 重构优化日志记录类
 * @version 2.0
 * @date 2025-09-10
 *
 * 1
 */

#ifndef  SUPPORT_COMMON_UTILS_INCLUDE_COMMON_UTILS_LOGGER_H_
#define  SUPPORT_COMMON_UTILS_INCLUDE_COMMON_UTILS_LOGGER_H_

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

namespace logger {
constexpr auto FuncFormat = "[{}:{}]:{}";
constexpr auto kDebug = "debug";
constexpr auto kInfo = "info";
constexpr auto kWarn = "warn";
constexpr auto kError = "error";
constexpr auto kFatal = "fatal";
const std::string log_dir_path_ = "/var/log/robot_log/";
constexpr auto kFlushInterval = 10; //s

class Logger {
 public:
  // 日志级别枚举
  enum class Level {
    DEBUG = spdlog::level::debug,
    INFO = spdlog::level::info,
    WARN = spdlog::level::warn,
    ERROR = spdlog::level::err,
    FATAL = spdlog::level::critical
  };

  // 节流结构体数据
  typedef struct ThrottleInfo {
    std::chrono::steady_clock::time_point last_log_time;  // 上一次打印日志的时间
    uint32_t duration;                                    // 输出日志的间隔
  } ThrottleInfo;

  typedef struct StLogThreadConf {
    std::size_t queue_size;
    std::size_t thread_count;
  } LogThreadConf;

  ~Logger();
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  // 初始化日志系统（整个进程调用一次）
  static void InitSystem(const rclcpp::Node::SharedPtr& node);

  // 配置spdlog异步线程池参数（需在InitSystem或首次GetInstance之前调用）
  static void SetThreadPoolConfig(std::size_t queue_size, std::size_t thread_count);

  // 创建或获取指定节点的日志器
  static std::shared_ptr<Logger> GetInstance(const std::string& log_name);

  // 关闭日志系统
  static void Shutdown();

  // 设置当前日志级别
  void SetLevel(Level level);

  // 刷新日志
  void Flush() { logger_->flush(); }

  // 新式日志记录接口（日志格式需要函数体，通过宏传入获取函数体）
  template <typename... Args>
  void Debug(const char* func, int line, fmt::format_string<Args...> fmt, Args&&... args) {
    logger_->debug(FuncFormat, func, line, fmt::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void Info(const char* func, int line, fmt::format_string<Args...> fmt, Args&&... args) {
    logger_->info(FuncFormat, func, line, fmt::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void Warn(const char* func, int line, fmt::format_string<Args...> fmt, Args&&... args) {
    logger_->warn(FuncFormat, func, line, fmt::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void Error(const char* func, int line, fmt::format_string<Args...> fmt, Args&&... args) {
    logger_->error(FuncFormat, func, line, fmt::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void Fatal(const char* func, int line, fmt::format_string<Args...> fmt, Args&&... args) {
    logger_->critical(FuncFormat, func, line, fmt::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void DebugThrottle(const char* func, int line, uint32_t duration, fmt::format_string<Args...> fmt, Args&&... args) {
    LogDuration(Level::DEBUG, func, line, duration, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void InfoThrottle(const char* func, int line, uint32_t duration, fmt::format_string<Args...> fmt, Args&&... args) {
    LogDuration(Level::INFO, func, line, duration, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void WarnThrottle(const char* func, int line, uint32_t duration, fmt::format_string<Args...> fmt, Args&&... args) {
    LogDuration(Level::WARN, func, line, duration, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void ErrorThrottle(const char* func, int line, uint32_t duration, fmt::format_string<Args...> fmt, Args&&... args) {
    LogDuration(Level::ERROR, func, line, duration, fmt, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void FatalThrottle(const char* func, int line, uint32_t duration, fmt::format_string<Args...> fmt, Args&&... args) {
    LogDuration(Level::FATAL, func, line, duration, fmt, std::forward<Args>(args)...);
  }

  // 兼容旧式日志接口（C风格可变参数）

  void Debug_C(const char* func, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    LogV(Level::DEBUG, func, fmt, args);
    va_end(args);
  }

  void Info_C(const char* func, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    LogV(Level::INFO, func, fmt, args);
    va_end(args);
  }

  void Warn_C(const char* func, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    LogV(Level::WARN, func, fmt, args);
    va_end(args);
  }

  void Error_C(const char* func, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    LogV(Level::ERROR, func, fmt, args);
    va_end(args);
  }

  void Fatal_C(const char* func, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    LogV(Level::FATAL, func, fmt, args);
    va_end(args);
  }

 private:
  Logger(const std::string& log_name, Level level, const rclcpp::Node::SharedPtr& config_node);

  // 从参数服务器获取节点配置
  void LoadConfig(const rclcpp::Node::SharedPtr& node);

  // 参数变更回调
  rcl_interfaces::msg::SetParametersResult OnParamChange(const std::vector<rclcpp::Parameter>& params);

  // 信号处理函数
  static void SignalHandler(int signal);

  static void ThreadConfig(LogThreadConf& conf, bool b_set = false);
  static std::atomic<bool>& SystemInitialized();
  static std::atomic<bool>& FlushSchedulerFlag();
  static rclcpp::Node::SharedPtr ConfNode(const rclcpp::Node::SharedPtr& node_ptr = nullptr, bool b_reset = false);
  static std::shared_ptr<Logger> LoggerInstance(const std::string& log_name, int sig = 0);

  // 节流日志记录
  template <typename... Args>
  void LogDuration(Level level, const char* func, int line, uint32_t duration,
                   fmt::format_string<Args...> fmt, Args&&... args) {
    if (level < current_level_) {
      return;
    }

    // 使用函数名+行号+格式模板作为节流key，而不是完整消息
    fmt::string_view fmt_view = fmt;
    std::string fmt_str(fmt_view.data(), fmt_view.size());
    std::string throttle_key = std::string(func) + ":" + std::to_string(line) + ":" + fmt_str;

    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> locker(throttle_mtx_);

    if (throttle_record_.count(throttle_key) == 0) {
      ThrottleInfo throttle_info;
      throttle_info.last_log_time = now;
      throttle_info.duration = duration;
      throttle_record_[throttle_key] = throttle_info;
    } else if (std::chrono::duration_cast<std::chrono::seconds>(now - throttle_record_[throttle_key].last_log_time)
                   .count() >= throttle_record_[throttle_key].duration) {
      throttle_record_[throttle_key].last_log_time = now;
    } else {
      return;
    }

    // 格式化消息
    std::string log_msg = fmt::format(fmt, std::forward<Args>(args)...);

    switch (level) {
      case Level::DEBUG:
        logger_->debug(FuncFormat, func, line, log_msg);
        break;
      case Level::INFO:
        logger_->info(FuncFormat, func, line, log_msg);
        break;
      case Level::WARN:
        logger_->warn(FuncFormat, func, line, log_msg);
        break;
      case Level::ERROR:
        logger_->error(FuncFormat, func, line, log_msg);
        break;
      case Level::FATAL:
        logger_->critical(FuncFormat, func, line, log_msg);
        break;
      default:
        logger_->info(FuncFormat, func, line, log_msg);
        break;
    }
  }

  // 可变参数日志实现
  void LogV(Level level, const char* func, const char* fmt, va_list args);

  std::shared_ptr<spdlog::async_logger> logger_;
  std::string log_name_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  Level current_level_;
  std::mutex throttle_mtx_;
  std::unordered_map<std::string, ThrottleInfo> throttle_record_;

  std::thread redis_log_thread_;
  std::mutex redis_log_mtx_;
  std::condition_variable redis_log_cond_;
  std::queue<std::string> redis_log_que_;

  bool is_running_ = true;
};

void GlobalLoggerInstance(std::shared_ptr<logger::Logger>& instance, bool b_set = false);

}  // namespace logger

// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define LOG_DEBUG(logger, fmt, ...) logger->Debug(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(logger, fmt, ...) logger->Info(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(logger, fmt, ...) logger->Warn(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(logger, fmt, ...) logger->Error(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_FATAL(logger, fmt, ...) logger->Fatal(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#define LogDebug(fmt, ...) Debug(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LogInfo(fmt, ...) Info(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LogWarn(fmt, ...) Warn(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LogError(fmt, ...) Error(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LogFatal(fmt, ...) Fatal(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#define LogThrottleDebug(duration, fmt, ...) DebugThrottle(__FUNCTION__, __LINE__, duration, fmt, ##__VA_ARGS__)
#define LogThrottleInfo(duration, fmt, ...) InfoThrottle(__FUNCTION__, __LINE__, duration, fmt, ##__VA_ARGS__)
#define LogThrottleWarn(duration, fmt, ...) WarnThrottle(__FUNCTION__, __LINE__, duration, fmt, ##__VA_ARGS__)
#define LogThrottleError(duration, fmt, ...) ErrorThrottle(__FUNCTION__, __LINE__, duration, fmt, ##__VA_ARGS__)
#define LogThrottleFatal(duration, fmt, ...) FatalThrottle(__FUNCTION__, __LINE__, duration, fmt, ##__VA_ARGS__)
// NOLINTEND(cppcoreguidelines-macro-usage)

// 兼容部分旧版日志接口的宏 暂定不再兼容
// #define LogDebug(fmt, ...) Debug_C(__FUNCTION__, fmt, ##__VA_ARGS__)
// #define LogInfo(fmt, ...) Info_C(__FUNCTION__, fmt, ##__VA_ARGS__)
// #define LogWarn(fmt, ...) Warn_C(__FUNCTION__, fmt, ##__VA_ARGS__)
// #define LogError(fmt, ...) Error_C(__FUNCTION__, fmt, ##__VA_ARGS__)
// #define LogFatal(fmt, ...) Fatal_C(__FUNCTION__, fmt, ##__VA_ARGS__)

// 设置当前的日志实例
inline void SetCurrentLogger(std::shared_ptr<logger::Logger> logger) { logger::GlobalLoggerInstance(logger, true); }

// 获取日志实例
inline std::shared_ptr<logger::Logger> GetLogger() {
  std::shared_ptr<logger::Logger> logger_instance = nullptr;
  logger::GlobalLoggerInstance(logger_instance);
  if (!logger_instance) {
    // 未设置日志指针，设置默认日志实例
    logger_instance = logger::Logger::GetInstance("default_log");
    logger::GlobalLoggerInstance(logger_instance, true);
  }
  return logger_instance;
}

// 日志宏定义
// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define DEBUG(fmt, ...) GetLogger()->Debug(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define INFO(fmt, ...) GetLogger()->Info(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define WARN(fmt, ...) GetLogger()->Warn(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define ERROR(fmt, ...) GetLogger()->Error(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define FATAL(fmt, ...) GetLogger()->Fatal(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#define THROTTLEDEBUG(duration, fmt, ...) GetLogger()->DebugThrottle(__FUNCTION__, __LINE__, \
  duration, fmt, ##__VA_ARGS__)
#define THROTTLEINFO(duration, fmt, ...) GetLogger()->InfoThrottle(__FUNCTION__, __LINE__, \
  duration, fmt, ##__VA_ARGS__)
#define THROTTLEWARN(duration, fmt, ...) GetLogger()->WarnThrottle(__FUNCTION__, __LINE__, \
  duration, fmt, ##__VA_ARGS__)
#define THROTTLEERROR(duration, fmt, ...) GetLogger()->ErrorThrottle(__FUNCTION__, __LINE__, \
  duration, fmt, ##__VA_ARGS__)
#define THROTTLEFATAL(duration, fmt, ...) GetLogger()->FatalThrottle(__FUNCTION__, __LINE__, \
  duration, fmt, ##__VA_ARGS__)

// NOLINTEND(cppcoreguidelines-macro-usage)

#endif  //  SUPPORT_COMMON_UTILS_INCLUDE_COMMON_UTILS_LOGGER_H_

/**
 * @file logger.cc
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "common_utils/logger.h"

#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <iostream>
#include <vector>
#include <string>
#include <cstdio>

#include "common_utils/common_utils.h"

namespace logger {

constexpr auto kDefPoolSize = 8192;

void Logger::ThreadConfig(LogThreadConf& conf, bool b_set) {
  static std::mutex mtx;
  static LogThreadConf global_conf{kDefPoolSize, 1};
  std::lock_guard<std::mutex> lock(mtx);
  if (b_set) {
    global_conf.queue_size = conf.queue_size;
    global_conf.thread_count = conf.thread_count;
  } else {
    conf.queue_size = global_conf.queue_size;
    conf.thread_count = global_conf.thread_count;
  }
}

std::atomic<bool>& Logger::SystemInitialized() {
  static std::atomic<bool> initialized{false};
  return initialized;
}

std::atomic<bool>& Logger::FlushSchedulerFlag() {
  static std::atomic<bool> scheduler_flag{false};
  return scheduler_flag;
}

rclcpp::Node::SharedPtr Logger::ConfNode(const rclcpp::Node::SharedPtr& node_ptr, bool b_reset) {
  static rclcpp::Node::SharedPtr conf_node = nullptr;
  static std::mutex mtx;
  std::lock_guard<std::mutex> lock(mtx);

  if (node_ptr) {
    conf_node = node_ptr;
  }

  if (b_reset && conf_node) {
    conf_node.reset();
  }

  return conf_node;
}

std::shared_ptr<Logger> Logger::LoggerInstance(const std::string& log_name, int sig) {
  static std::unordered_map<std::string, std::weak_ptr<Logger>> logger_map;
  static std::mutex map_mutex;

  std::lock_guard<std::mutex> lock(map_mutex);
  if (!log_name.empty()) {
    auto it = logger_map.find(log_name);
    if (it != logger_map.end()) {
      if (auto logger = it->second.lock()) {
        return logger;
      }
    }

    // 创建新实例
    auto new_logger = std::shared_ptr<Logger>(new Logger(log_name, Level::INFO, ConfNode()));
    logger_map[log_name] = new_logger;
    return new_logger;
  }

  for (const auto& map_ele : logger_map) {
    if (auto logger = map_ele.second.lock()) {
      try {
        if (sig != 0) {
          LOG_FATAL(logger, "Program received signal: {}", sig);
        }
        logger->Flush();
      } catch (...) {
      }
    }
  }

  if (sig == 0) {
    logger_map.clear();
  }

  return nullptr;
}

void Logger::InitSystem(const rclcpp::Node::SharedPtr& node) {
  if (SystemInitialized() || !node) {
    return;
  }

  LogThreadConf thread_conf{kDefPoolSize, 1};
  ThreadConfig(thread_conf);
  // 存储配置节点
  ConfNode(node);

  // 初始化异步日志池
  spdlog::init_thread_pool(thread_conf.queue_size, thread_conf.thread_count);

  // 启动全局定时刷新（对已注册到registry的logger生效）
  if (!FlushSchedulerFlag().exchange(true)) {
    spdlog::flush_every(std::chrono::seconds(kFlushInterval));
  }

  // 注册信号处理
  // std::signal(SIGINT, SignalHandler);
  std::signal(SIGABRT, SignalHandler);
  std::signal(SIGSEGV, SignalHandler);
  std::signal(SIGFPE, SignalHandler);

  SystemInitialized().store(true);
  std::cout << "LoggerSyetem initialized by node:" << node->get_name()
            << " thread_pool_queue_size:" << thread_conf.queue_size
            << " thread_pool_thread_count:" << thread_conf.thread_count << std::endl;
}

void Logger::Shutdown() {
  if (!SystemInitialized()) {
    return;
  }

  std::cout << "LoggerSyetem Shutdown" << std::endl;

  // 清理所有已创建的logger并关闭spdlog线程池
  LoggerInstance("");

  if (ConfNode()) {
    ConfNode(nullptr, true);
  }

  // 标记系统关闭，后续析构不再尝试写日志
  SystemInitialized().store(false);

  // 先丢弃注册的logger，再销毁线程池
  try {
    spdlog::drop_all();
  } catch (...) {
  }
  try {
    spdlog::shutdown();
  } catch (...) {
  }
}

logger::Logger::~Logger() {
  is_running_ = false;
  if (redis_log_thread_.joinable()) {
    redis_log_thread_.join();
  }
  // 系统关闭后不要再写日志，避免线程池已销毁导致异常
  if (!SystemInitialized()) {
    return;
  }

  try {
    if (logger_) {
      logger_->flush();
    }
  } catch (...) {
  }
}

std::shared_ptr<Logger> Logger::GetInstance(const std::string& log_name) {
  static std::mutex mtx;
  std::lock_guard<std::mutex> lock(mtx);
  // 增加非节点日志打印支持，配置下spdlog异步设置
  if (!SystemInitialized()) {
    LogThreadConf thread_conf{kDefPoolSize, 1};
    ThreadConfig(thread_conf);
    // 初始化异步日志池
    spdlog::init_thread_pool(thread_conf.queue_size, thread_conf.thread_count);

    // 注册信号处理
    // std::signal(SIGINT, SignalHandler);
    std::signal(SIGABRT, SignalHandler);
    std::signal(SIGSEGV, SignalHandler);
    std::signal(SIGFPE, SignalHandler);

    SystemInitialized().store(true);
    // 启动全局定时刷新（对已注册到registry的logger生效）
    if (!FlushSchedulerFlag().exchange(true)) {
      spdlog::flush_every(std::chrono::seconds(kFlushInterval));
    }
    std::cout << "LoggerSyetem initialized by log_name:" << log_name
              << " thread_pool_queue_size:" << thread_conf.queue_size
              << " thread_pool_thread_count:" << thread_conf.thread_count << std::endl;
  }

  return LoggerInstance(log_name);
}

void Logger::SetThreadPoolConfig(std::size_t queue_size, std::size_t thread_count) {
  // 只能在线程池初始化之前设置，避免重复初始化spdlog线程池
  if (SystemInitialized()) {
    std::cout << "Logger thread pool already initialized, SetThreadPoolConfig will be ignored." << std::endl;
    return;
  }
  LogThreadConf thread_conf{kDefPoolSize, 1};
  ThreadConfig(thread_conf);

  if (queue_size == 0 || thread_count == 0) {
    std::cout << "Logger SetThreadPoolConfig invalid param, keep default: queue_size="
              << thread_conf.queue_size << ", thread_count=" << thread_conf.thread_count << std::endl;
    return;
  }
  thread_conf.queue_size = queue_size;
  thread_conf.thread_count = thread_count;
  ThreadConfig(thread_conf, true);
}

Logger::Logger(const std::string& log_name, Level level, const rclcpp::Node::SharedPtr& config_node)
    : log_name_(log_name), current_level_(level) {
  // 从参数服务器获取配置
  if (config_node) {
    // 声明节点专属参数
    std::string param_name = log_name_ + ".log_level";
    if (!config_node->has_parameter(param_name)) {
      // 声明节点专属日志级别参数
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = "Log level for " + log_name_ + " log_node";
      descriptor.read_only = false;

      // 设置允许的值
      std::vector<std::string> allowed_values = {kDebug, kInfo, kWarn, kError, kFatal};
      std::string allowed_str;
      for (const auto& v : allowed_values) {
        if (!allowed_str.empty()) {
          allowed_str += ", ";
        }
        allowed_str += v;
      }
      descriptor.additional_constraints = "Allowed values: " + allowed_str;

      // 将当前级别转换为字符串
      std::string level_str = kInfo;
      switch (current_level_) {
        case Level::DEBUG:
          level_str = kDebug;
          break;
        case Level::INFO:
          level_str = kInfo;
          break;
        case Level::WARN:
          level_str = kWarn;
          break;
        case Level::ERROR:
          level_str = kError;
          break;
        case Level::FATAL:
          level_str = kFatal;
          break;
      }

      config_node->declare_parameter(param_name, level_str, descriptor);
    }

    // 加载配置
    LoadConfig(config_node);
  }

  // 创建异步日志器
  // 后续轮转采用logrotate系统工具定时压缩打包实现，basic_file_sink_mt第二个参数false表示追加模式，true为每次启动覆盖原来日志文件
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_dir_path_ + log_name + ".log");

  logger_ = std::make_shared<spdlog::async_logger>(log_name, file_sink, spdlog::thread_pool(),
                                                   spdlog::async_overflow_policy::block);

  // 设置日志格式：[级别][时间][进程ID][线程ID]  [函数名]消息
  logger_->set_pattern("[%l][%Y-%m-%d %H:%M:%S.%e][%P][%t]%v");

  // 设置日志级别和刷新策略
  logger_->set_level(static_cast<spdlog::level::level_enum>(current_level_));
  logger_->flush_on(spdlog::level::warn);  // WARN及以上级别立即刷新

  // 注册到registry以便flush_every生效
  try {
    spdlog::register_logger(logger_);
  } catch (...) {
  }

  // 注册参数变更回调
  if (ConfNode()) {
    param_callback_handle_ =
        ConfNode()->add_on_set_parameters_callback(std::bind(&Logger::OnParamChange, this, std::placeholders::_1));
  }
}

void Logger::LoadConfig(const rclcpp::Node::SharedPtr& node) {
  if (!node) {
    return;
  }

  // 读取节点专属日志级别
  std::string param_name = log_name_ + ".log_level";
  if (node->has_parameter(param_name)) {
    std::string level_str = node->get_parameter(param_name).as_string();
    try {
      if (level_str == kDebug) {
        current_level_ = Level::DEBUG;
      } else if (level_str == kInfo) {
        current_level_ = Level::INFO;
      } else if (level_str == kWarn) {
        current_level_ = Level::WARN;
      } else if (level_str == kError) {
        current_level_ = Level::ERROR;
      } else if (level_str == kFatal) {
        current_level_ = Level::FATAL;
      }
    } catch (...) {
    }
  }
}

void Logger::SetLevel(Level level) {
  current_level_ = level;
  logger_->set_level(static_cast<spdlog::level::level_enum>(level));
  logger_->critical("OnParamChange SetLogLevel:{} in logfile:{}", static_cast<int>(level), log_name_);
}

rcl_interfaces::msg::SetParametersResult Logger::OnParamChange(const std::vector<rclcpp::Parameter>& params) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  std::string param_name = log_name_ + ".log_level";
  for (const auto& param : params) {
    if (param.get_name() == param_name) {
      const std::string& level_str = param.as_string();
      try {
        if (level_str == kDebug) {
          SetLevel(Level::DEBUG);
        } else if (level_str == kInfo) {
          SetLevel(Level::INFO);
        } else if (level_str == kWarn) {
          SetLevel(Level::WARN);
        } else if (level_str == kError) {
          SetLevel(Level::ERROR);
        } else if (level_str == kFatal) {
          SetLevel(Level::FATAL);
        } else {
          result.successful = false;
        }
      } catch (...) {
        result.successful = false;
      }
      break;
    }
  }
  return result;
}

void Logger::LogV(Level level, const char* func, const char* fmt, va_list args) {
  // 确定格式化字符串长度
  va_list args_copy;
  va_copy(args_copy, args);
  int length = std::vsnprintf(nullptr, 0, fmt, args_copy);
  va_end(args_copy);

  if (length <= 0) {
    return;
  }

  // 创建缓冲区并格式化字符串
  std::vector<char> buf(length + 1);
  std::vsnprintf(buf.data(), buf.size(), fmt, args);

  // 根据日志级别记录
  switch (level) {
    case Level::DEBUG:
      logger_->debug(FuncFormat, func, buf.data());
      break;
    case Level::INFO:
      logger_->info(FuncFormat, func, buf.data());
      break;
    case Level::WARN:
      logger_->warn(FuncFormat, func, buf.data());
      break;
    case Level::ERROR:
      logger_->error(FuncFormat, func, buf.data());
      break;
    case Level::FATAL:
      logger_->critical(FuncFormat, func, buf.data());
      break;
    default:
      logger_->info(FuncFormat, func, buf.data());
      break;
  }
}

void Logger::SignalHandler(int signal) {
  // 刷新所有日志
  LoggerInstance("", signal);

  // 重新抛出信号
  std::signal(signal, SIG_DFL);
  std::raise(signal);
  // std::_Exit(signal);
}

void GlobalLoggerInstance(std::shared_ptr<logger::Logger>& instance, bool b_set) {
  static std::mutex mtx;
  static std::shared_ptr<logger::Logger> logger_instance;
  std::lock_guard<std::mutex> lock(mtx);

  if (b_set) {
    logger_instance = instance;
  } else {
    instance = logger_instance;
  }
}

}  // namespace logger

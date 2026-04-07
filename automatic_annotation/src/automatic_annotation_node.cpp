/**
 * @file magnetometer_driver_node.cpp
 * @author KY.LI (keyao.li@bynav.com)
 * @brief 
 * @version 1.0
 * @date 2025-11-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <exception>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common_utils/logger.h"
#include "automatic_annotation/gen_prompt_point.hpp"

int main(int argc, char* argv[]) {
  try {
    rclcpp::init(argc, argv);
    rclcpp::ExecutorOptions defaultOptions;
    rclcpp::executors::MultiThreadedExecutor executor(defaultOptions);
    auto node = std::make_shared<rclcpp::Node>("automatic_annotation_node");
    logger::Logger::InitSystem(node);
    auto log_p = logger::Logger::GetInstance("automatic_annotation_log");
    SetCurrentLogger(log_p);
    INFO("");
    auto gen_prompt_point = std::make_shared<automatic_annotation::GenPromptPoint>(node);
    executor.add_node(node);
    executor.spin();
    INFO("after spin...");
    gen_prompt_point.reset();  // 触发析构函数
    logger::Logger::Shutdown();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    // 捕获标准库异常
    std::cerr << "Unhandled exception in main: " << e.what() << std::endl;
    // 确保ROS 2正确关闭
    if (rclcpp::ok()) {
      logger::Logger::Shutdown();
      rclcpp::shutdown();
    }
    return 1;
  } catch (...) {
    // 捕获任何其他未知类型的异常
    std::cerr << "Unknown unhandled exception in main." << std::endl;
    // 确保ROS 2正确关闭
    if (rclcpp::ok()) {
      logger::Logger::Shutdown();
      rclcpp::shutdown();
    }
    return 2;
  }
}

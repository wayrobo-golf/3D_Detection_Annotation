cmake_minimum_required(VERSION 3.5)
# 修复 JsonCpp 策略报错：强制要求至少 3.5 版本的策略 
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
project(common_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string(TOLOWER "$ENV{ROBOT_TYPE}" rob_type)
message("${rob_type}")

# 1. 移除了 robot_msgs 依赖 
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  rclcpp_lifecycle
  yaml-cpp
  spdlog
  fmt
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp REQUIRED)

option(ENABLE_GPERFTOOLS "Enable gperftools heap/cpu profiler in common_utils" OFF)

# 2. 修正路径：根据你的 ls 结果，redis-plus-plus 在 ../ 目录下 
set(REDIS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../redis-plus-plus)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  set(REDIS_LIB ${REDIS_ROOT}/lib/arm64/libredis++.a)
  set(HIREDIS_LIB ${REDIS_ROOT}/lib/arm64/libhiredis.a)
  # 如果 gperftools 也在同级目录下，请参照此逻辑修改。目前维持原样或建议检查路径。
  set(GPERFTOOLS_LIB ${CMAKE_SOURCE_DIR}/../gperftools-2.17/lib/arm64/libtcmalloc_and_profiler.a)
else()
  set(REDIS_LIB ${REDIS_ROOT}/lib/amd64/libredis++.a)
  set(HIREDIS_LIB ${REDIS_ROOT}/lib/amd64/libhiredis.a)
  set(GPERFTOOLS_LIB ${CMAKE_SOURCE_DIR}/../gperftools-2.17/lib/amd64/libtcmalloc_and_profiler.a)
endif()

add_library(${PROJECT_NAME} SHARED
  src/common_utils.cc
  src/logger.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # 修正头文件包含路径 [cite: 2]
  $<BUILD_INTERFACE:${REDIS_ROOT}/include>
  $<BUILD_INTERFACE:${ament_INCLUDE_DIRS}>
)

# 兼容不同发行版/打包方式
if(TARGET JsonCpp::JsonCpp)
  set(JSONCPP_TARGET JsonCpp::JsonCpp)
elseif(TARGET jsoncpp_lib)
  set(JSONCPP_TARGET jsoncpp_lib)
else()
  message(FATAL_ERROR "jsoncpp found but no CMake target exported.")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  yaml-cpp
  spdlog::spdlog
  fmt::fmt
  PRIVATE ${JSONCPP_TARGET} ${REDIS_LIB} ${HIREDIS_LIB}
)

if(ENABLE_GPERFTOOLS)
  target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../gperftools-2.17/include>
  )
  target_link_libraries(${PROJECT_NAME} PRIVATE
    ${GPERFTOOLS_LIB}
    unwind
    dl
    pthread
  )	
else()		
  message(STATUS "common_utils: gperftools heap/cpu profiler DISABLED")		
endif()

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies} CURL)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

install(DIRECTORY include/
  DESTINATION include
)

if (rob_type STREQUAL "robot_mower")
  file(GLOB CONFIG_FILES "config/*")
  install(FILES ${CONFIG_FILES} DESTINATION share/${PROJECT_NAME}/config/)
endif()

ament_package()
#ifndef AUTOMATIC_ANNOTATION_CONST_VALUE_HPP
#define AUTOMATIC_ANNOTATION_CONST_VALUE_HPP
#include <cstddef>
#include <Eigen/Dense>
namespace automatic_annotation {
  namespace ConstValue {
    constexpr size_t kLidarQueueSize = 5;
    constexpr size_t kCameraQueueSize = 1;
    constexpr size_t kUndistortedLidarQueueSize = 1;
    constexpr uint32_t kPointCloudTypeOriginal = 0;
    constexpr uint32_t kPointCloudTypeUndistorted = 1;
    constexpr uint32_t kImageSourceLeftCamera = 0;
    constexpr uint32_t kImageSourceRightCamera = 1;
    constexpr uint32_t kDefaultImageSaveInterval = 10;
    constexpr uint32_t kDefaultSizeofIntrinsic = 9;
    constexpr uint32_t kTargetPointCloudAccumulate = 10;
    constexpr uint32_t kCheckTfPeriodMs = 500;
    constexpr int32_t kThresholdPcCheck = 50;
    constexpr double kLidarToCameraMaxTimeDiff = 0.02;
    constexpr double kDefaultTfWaitTimeoutSec = 3.0;
    constexpr double kRad2Deg = 180.0 / M_PI;
    constexpr double kSeconds2Milliseconds = 1000.0;
    constexpr bool kDefaultPublishAccumulatedPC = false;
    constexpr bool kDefaultAutoAnnotation = false;
    constexpr bool kDefaultAnnotationSemantic = false;
    constexpr const char* kPackageName = "automatic_annotation";
    constexpr const char* kVersion = "1.0.0(2026-02-05)";
    constexpr const char* kDefaultOriginPointCloudTopic = "livox/pointcloud";
    constexpr const char* kDefaultUndistortedPointCloudTopic = "lio/world_pc";
    constexpr const char* kDefaultLeftImageTopic = "camera/left/image";
    constexpr const char* kDefaultRightImageTopic = "camera/right/image";
    constexpr const char* kDefaultLeftCameraIntrinsicTopic = "camera/left/intrinsic_matrix";
    constexpr const char* kDefaultRightCameraIntrinsicTopic = "camera/right/intrinsic_matrix";
    constexpr const char* kDefaultLeftCameraFigSaveFolder = "data_record/cam0";
    constexpr const char* kDefaultRightCameraFigSaveFolder = "data_record/cam1";
    constexpr const char* kDefaultAccumulatePointCloudSaveFolder = "data_record/point_cloud";
    constexpr const char* kInsLink = "ins_link";
    constexpr const char* kInsMap = "ins_map";
    constexpr const char* kLidarLink = "lidar_link";
    constexpr const char* kCameraLeftLink = "camera_infra1_optical_frame";
    constexpr const char* kCameraRightLink = "rcam_link";
    constexpr const char* kDefaultGlobalPCMapAddr = "";
    constexpr const char* kDefaultPCAnnotationFileAddr = "";
  }  // namespace ConstValue
}  // namespace automatic_annotation
#endif  // AUTOMATIC_ANNOTATION_CONST_VALUE_HPP
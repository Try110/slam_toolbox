/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include <memory>
#include <string>
#include "slam_toolbox/slam_toolbox_localization.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
LocalizationSlamToolbox::LocalizationSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
  processor_type_ = PROCESS_LOCALIZATION;
  localization_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(&LocalizationSlamToolbox::localizePoseCallback,
    this, std::placeholders::_1));
  clear_localization_ = this->create_service<std_srvs::srv::Empty>(
    "slam_toolbox/clear_localization_buffer",
    std::bind(&LocalizationSlamToolbox::clearLocalizationBuffer, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // in localization mode, we cannot allow for interactive mode
  enable_interactive_mode_ = false;

  // in localization mode, disable map saver
  map_saver_.reset();
}

/*****************************************************************************/
void LocalizationSlamToolbox::loadPoseGraphByParams()
/*****************************************************************************/
{
  std::string filename;
  geometry_msgs::msg::Pose2D pose;
  bool dock = false;
  if (shouldStartWithPoseGraph(filename, pose, dock)) {
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req =
      std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp =
      std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Response>();
    req->initial_pose = pose;
    req->filename = filename;
    req->match_type =
      slam_toolbox::srv::DeserializePoseGraph::Request::LOCALIZE_AT_POSE;
    if (dock) {
      RCLCPP_WARN(get_logger(),
        "LocalizationSlamToolbox: Starting localization "
        "at first node (dock) is correctly not supported.");
    }

    deserializePoseGraphCallback(nullptr, req, resp);
  }
}

/*****************************************************************************/
bool LocalizationSlamToolbox::clearLocalizationBuffer(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> resp)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(smapper_mutex_);
  RCLCPP_INFO(get_logger(),
    "LocalizationSlamToolbox: Clearing localization buffer.");
  smapper_->clearLocalizationBuffer();
  return true;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::serializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp)
/*****************************************************************************/
{
  RCLCPP_ERROR(get_logger(), "LocalizationSlamToolbox: Cannot call serialize map "
    "in localization mode!");
  return false;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type != procType::LOCALIZE_AT_POSE) {
    RCLCPP_ERROR(get_logger(), "Requested a non-localization deserialization "
      "in localization mode.");
    return false;
  }
  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

/*****************************************************************************/
void LocalizationSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  // store scan header
  scan_header = scan->header;
  // no odom info
  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(get_logger(), "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  if (shouldProcessScan(scan, pose)) {
    addScan(laser, scan, pose);
  }
}

/*****************************************************************************/
/**
 * 在定位模式下处理激光扫描数据
 * @param laser 激光雷达设备对象
 * @param scan ROS激光扫描消息的共享指针
 * @param odom_pose 从里程计获取的位姿
 * @return 处理后的LocalizedRangeScan对象指针，如果处理失败则返回nullptr
 */
LocalizedRangeScan * LocalizationSlamToolbox::addScan(
  LaserRangeFinder * laser,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  Pose2 & odom_pose)
/*****************************************************************************/
{
  // 获取位姿互斥锁，确保线程安全
  boost::mutex::scoped_lock l(pose_mutex_);

  // 如果当前是定位模式且有待处理的近区位姿请求，则切换到近区处理模式
  if (processor_type_ == PROCESS_LOCALIZATION && process_near_pose_) {
    processor_type_ = PROCESS_NEAR_REGION;
  }

  // 将ROS激光扫描数据转换为Karto系统的LocalizedRangeScan对象
  LocalizedRangeScan * range_scan = getLocalizedRangeScan(
    laser, scan, odom_pose);

  // 获取SLAM映射器互斥锁，确保线程安全
  boost::mutex::scoped_lock lock(smapper_mutex_);

  // 处理结果标志和变换更新标志
  bool processed = false, update_reprocessing_transform = false;

  // 初始化协方差矩阵为单位矩阵
  Matrix3 covariance;
  covariance.SetToIdentity();

  // 根据处理器类型执行不同的处理逻辑,定位初始化
  if (processor_type_ == PROCESS_NEAR_REGION) {
    // 近区处理模式（当用户请求在特定区域进行定位时）
    if (!process_near_pose_) {
      // 如果没有有效的近区位姿请求，则报错并返回
      RCLCPP_ERROR(get_logger(),
        "Process near region called without a "
        "valid region request. Ignoring scan.");
      return nullptr;
    }

    // 将扫描的里程计位姿设置为请求的近区位姿
    range_scan->SetOdometricPose(*process_near_pose_);
    // 将校正位姿也设置为相同的值（初始估计）
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    // 重置近区位姿请求
    process_near_pose_.reset(nullptr);

    // 使用近区节点处理方法处理扫描，将扫描添加到定位缓冲区
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan, true, &covariance);

    // 需要更新重处理变换
    update_reprocessing_transform = true;
    // 处理完成后重置为定位模式
    processor_type_ = PROCESS_LOCALIZATION;
  } else if (processor_type_ == PROCESS_LOCALIZATION) {
    // 标准定位模式处理
    processed = smapper_->getMapper()->ProcessLocalization(range_scan, &covariance);
    update_reprocessing_transform = false;
  } else {
    // 处理器类型错误，记录致命错误并退出
    RCLCPP_FATAL(get_logger(), "LocalizationSlamToolbox: "
      "No valid processor type set! Exiting.");
    exit(-1);
  }

  // 如果处理失败，删除扫描对象并返回空指针
  if (!processed) {
    delete range_scan;
    range_scan = nullptr;
  } else {
    // 处理成功，计算并发布从里程计到地图的变换
    setTransformFromPoses(range_scan->GetCorrectedPose(), odom_pose,
      scan->header.stamp, update_reprocessing_transform);

    // 发布当前位姿估计和协方差
    publishPose(range_scan->GetCorrectedPose(), covariance, scan->header.stamp);
  }

  // 返回处理后的扫描对象
  return range_scan;
}


/*****************************************************************************/
void LocalizationSlamToolbox::localizePoseCallback(
  const
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION) {
    RCLCPP_ERROR(get_logger(),
      "LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  boost::mutex::scoped_lock l(pose_mutex_);
  if (process_near_pose_) {
    process_near_pose_.reset(new Pose2(msg->pose.pose.position.x,
      msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation)));
  } else {
    process_near_pose_ = std::make_unique<Pose2>(msg->pose.pose.position.x,
        msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
  }

  first_measurement_ = true;

  boost::mutex::scoped_lock lock(smapper_mutex_);
  smapper_->clearLocalizationBuffer();

  RCLCPP_INFO(get_logger(),
    "LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    tf2::getYaw(msg->pose.pose.orientation));
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::LocalizationSlamToolbox)

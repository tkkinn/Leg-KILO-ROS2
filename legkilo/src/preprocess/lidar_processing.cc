#include "preprocess/lidar_processing.h"

namespace legkilo {

namespace {
inline double stampToSec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}
}  // namespace

LidarProcessing::LidarProcessing(LidarProcessing::Config config) : config_(config) {
    LOG(INFO) << "Lidar Processing is Constructed";
    cloud_pcl_.reset(new PointCloudType());
}

LidarProcessing::~LidarProcessing() { LOG(INFO) << "Lidar Processing is Destructed"; }

common::LidarType LidarProcessing::getLidarType() const { return config_.lidar_type_; }

void LidarProcessing::processing(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, common::LidarScan& lidar_scan) {
    switch (config_.lidar_type_) {
        case common::LidarType::VEL: velodyneHandler(msg, lidar_scan); break;

        case common::LidarType::OUSTER: ousterHander(msg, lidar_scan); break;

        case common::LidarType::HESAI: hesaiHandler(msg, lidar_scan); break;

        case common::LidarType::LIVOX:
            LOG(ERROR) << "LIVOX lidar type expects livox_interfaces::msg::CustomMsg";
            break;

        default: LOG(ERROR) << " Lidar Type is Not Currently Available"; break;
    }
}

void LidarProcessing::processing(const livox_interfaces::msg::CustomMsg::ConstSharedPtr& msg,
                                 common::LidarScan& lidar_scan) {
    if (config_.lidar_type_ != common::LidarType::LIVOX) {
        LOG(WARNING) << "Received Livox CustomMsg while lidar_type is set to " << static_cast<int>(config_.lidar_type_)
                     << ". Processing it as LIVOX.";
    }
    livoxHandler(msg, lidar_scan);
}

void LidarProcessing::velodyneHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                                      common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    pcl::PointCloud<velodyne_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    float first_point_time = config_.time_scale_ * cloud_pcl_raw.points.front().time;
    float last_point_time = config_.time_scale_ * cloud_pcl_raw.points.back().time;

    const double stamp_sec = stampToSec(msg->header.stamp);
    lidar_scan.lidar_begin_time_ = stamp_sec + first_point_time;
    lidar_scan.lidar_end_time_ = stamp_sec + last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || blindCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        float cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].time;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0f) / 500.0f;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::ousterHander(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                                   common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());
    pcl::PointCloud<ouster_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    float first_point_time = config_.time_scale_ * cloud_pcl_raw.points.front().t;
    float last_point_time = config_.time_scale_ * cloud_pcl_raw.points.back().t;

    const double stamp_sec = stampToSec(msg->header.stamp);
    lidar_scan.lidar_begin_time_ = stamp_sec + first_point_time;
    lidar_scan.lidar_end_time_ = stamp_sec + last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || blindCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        float cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].t;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0f) / 500.0f;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::hesaiHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                                   common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());
    pcl::PointCloud<hesai_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    double first_point_time = config_.time_scale_ * cloud_pcl_raw.points.front().timestamp;
    double last_point_time = config_.time_scale_ * cloud_pcl_raw.points.back().timestamp;

    lidar_scan.lidar_begin_time_ = first_point_time;
    lidar_scan.lidar_end_time_ = last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || blindCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        double cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].timestamp;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0f) / 500.0f;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}

void LidarProcessing::livoxHandler(const livox_interfaces::msg::CustomMsg::ConstSharedPtr& msg,
                                   common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    const double stamp_sec = stampToSec(msg->header.stamp);
    if (msg->points.empty()) {
        lidar_scan.lidar_begin_time_ = stamp_sec;
        lidar_scan.lidar_end_time_ = stamp_sec;
        return;
    }

    const int cloud_size = static_cast<int>(msg->points.size());
    lidar_scan.cloud_->points.reserve(cloud_size);

    uint32_t valid_num = 0;
    double first_point_time = -1.0;
    double last_point_time = 0.0;

    for (const auto& pt : msg->points) {
        const uint8_t tag_state = static_cast<uint8_t>(pt.tag & 0x30);
        if (!(tag_state == 0x10 || tag_state == 0x00)) continue;

        ++valid_num;
        if (valid_num % static_cast<uint32_t>(config_.filter_num_) != 0U) continue;

        PointType added_point;
        added_point.x = pt.x;
        added_point.y = pt.y;
        added_point.z = pt.z;
        added_point.intensity = static_cast<float>(pt.reflectivity);
        if (blindCheck(added_point)) continue;

        const double point_time = config_.time_scale_ * static_cast<double>(pt.offset_time);
        if (first_point_time < 0.0) { first_point_time = point_time; }
        last_point_time = point_time;

        added_point.curvature = std::round((point_time - first_point_time) * 500.0) / 500.0;
        lidar_scan.cloud_->points.push_back(added_point);
    }

    if (first_point_time < 0.0) {
        lidar_scan.lidar_begin_time_ = stamp_sec;
        lidar_scan.lidar_end_time_ = stamp_sec;
        return;
    }

    lidar_scan.lidar_begin_time_ = stamp_sec + first_point_time;
    lidar_scan.lidar_end_time_ = stamp_sec + last_point_time;
}

}  // namespace legkilo

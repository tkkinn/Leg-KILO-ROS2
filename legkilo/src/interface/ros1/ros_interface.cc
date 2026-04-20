#include "interface/ros1/ros_interface.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <utility>

#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/KILO.h"
#include "preprocess/kinematics.h"
#include "preprocess/lidar_processing.h"

namespace legkilo {

namespace {

inline double stampToSec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

inline builtin_interfaces::msg::Time secToStamp(double sec) {
    builtin_interfaces::msg::Time stamp;
    const auto whole_sec = static_cast<int32_t>(std::floor(sec));
    const double frac_sec = sec - static_cast<double>(whole_sec);
    stamp.sec = whole_sec;
    stamp.nanosec = static_cast<uint32_t>(std::llround(frac_sec * 1e9));
    if (stamp.nanosec >= 1000000000U) {
        stamp.sec += 1;
        stamp.nanosec -= 1000000000U;
    }
    return stamp;
}

}  // namespace

RosInterface::RosInterface(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    LOG(INFO) << "Ros Interface is being Constructed";
    pub_odom_world_ = node_->create_publisher<nav_msgs::msg::Odometry>("/Odomtry", rclcpp::QoS(10000));
    pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(10000));
    pub_pointcloud_world_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", rclcpp::QoS(10000));
    pub_pointcloud_body_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", rclcpp::QoS(10000));
    pub_joint_state_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10000));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    odom_world_.header.frame_id = "camera_init";
    odom_world_.child_frame_id = "base";
    path_world_.header.frame_id = "camera_init";
    path_world_.header.stamp = node_->now();
    pose_path_.header.frame_id = "camera_init";
    tf_msg_.header.frame_id = "camera_init";
    tf_msg_.child_frame_id = "base";
}

RosInterface::~RosInterface() {
    LOG(INFO) << "Ros Interface is being Destructed";
    sub_lidar_raw_.reset();
    sub_imu_raw_.reset();
    sub_kinematic_raw_.reset();
}

bool RosInterface::initParamAndReset(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    /* Topic and options*/
    options::kLidarTopic = yaml_helper.get<std::string>("lidar_topic");
    options::kImuUse = yaml_helper.get<bool>("only_imu_use", true);
    options::kKinAndImuUse = static_cast<bool>(!options::kImuUse);
    options::kRedundancy = yaml_helper.get<bool>("redundancy", false);
    if (options::kImuUse) { options::kImuTopic = yaml_helper.get<std::string>("imu_topic"); }
    if (options::kKinAndImuUse) { options::kKinematicTopic = yaml_helper.get<std::string>("kinematic_topic"); }

    /* Odometry core (KILO) */
    kilo_ = std::make_unique<KILO>(config_file);

    /* kinematics*/
    Kinematics::Config kinematics_config;
    kinematics_config.leg_offset_x = yaml_helper.get<double>("leg_offset_x");
    kinematics_config.leg_offset_y = yaml_helper.get<double>("leg_offset_y");
    kinematics_config.leg_calf_length = yaml_helper.get<double>("leg_calf_length");
    kinematics_config.leg_thigh_length = yaml_helper.get<double>("leg_thigh_length");
    kinematics_config.leg_thigh_offset = yaml_helper.get<double>("leg_thigh_offset");
    kinematics_config.contact_force_threshold_up = yaml_helper.get<double>("contact_force_threshold_up");
    kinematics_config.contact_force_threshold_down = yaml_helper.get<double>("contact_force_threshold_down");
    kinematics_ = std::make_unique<Kinematics>(kinematics_config);

    /* lidar processing*/
    LidarProcessing::Config lidar_process_config;
    lidar_process_config.blind_ = yaml_helper.get<float>("blind");
    lidar_process_config.filter_num_ = yaml_helper.get<int>("filter_num");
    lidar_process_config.time_scale_ = yaml_helper.get<double>("time_scale");
    lidar_process_config.point_stamp_correct_ = yaml_helper.get<bool>("point_stamp_correct", false);
    lidar_process_config.lidar_type_ = static_cast<common::LidarType>(yaml_helper.get<int>("lidar_type"));
    lidar_processing_ = std::make_unique<LidarProcessing>(lidar_process_config);

    /* Visualizaition*/
    pub_joint_tf_enable_ = yaml_helper.get<bool>("pub_joint_tf_enable");

    /* Trajectory saving */
    const bool save_traj_enable = yaml_helper.get<bool>("save_traj_enable", false);
    if (save_traj_enable) { traj_saver_ = std::make_unique<TrajectorySaver>(); }

    /* PCD saving */
    const bool save_pcd_enable = yaml_helper.get<bool>("save_pcd_enable", false);
    const int pcd_frames_per_file = yaml_helper.get<int>("pcd_frames_per_file", 100);
    const double pcd_voxel_leaf = yaml_helper.get<double>("pcd_voxel_leaf_size", 0.1);
    if (save_pcd_enable) { pcd_saver_ = std::make_unique<PcdSaver>(pcd_frames_per_file, pcd_voxel_leaf); }

    return true;
}

void RosInterface::rosInit(const std::string& config_file) {
    this->initParamAndReset(config_file);
    this->subscribeLidar();

    if (options::kImuUse) { this->subscribeImu(); }

    if (options::kKinAndImuUse) { this->subscribeKinematicImu(); }
}

void RosInterface::subscribeLidar() {
    this->sub_lidar_raw_ =
        node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            options::kLidarTopic,
            rclcpp::SensorDataQoS(),
            std::bind(&RosInterface::lidarCallBack, this, std::placeholders::_1));
}

void RosInterface::subscribeImu() {
    this->sub_imu_raw_ =
        node_->create_subscription<sensor_msgs::msg::Imu>(
            options::kImuTopic,
            rclcpp::SensorDataQoS(),
            std::bind(&RosInterface::imuCallBack, this, std::placeholders::_1));
}

void RosInterface::subscribeKinematicImu() {
    this->sub_kinematic_raw_ =
        node_->create_subscription<unitree_legged_msgs::msg::HighState>(
            options::kKinematicTopic,
            rclcpp::SensorDataQoS(),
            std::bind(&RosInterface::kinematicImuCallBack, this, std::placeholders::_1));
}

void RosInterface::lidarCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double stamp_sec = stampToSec(msg->header.stamp);
    static double last_scan_time = stamp_sec;

    Timer::measure("Lidar Processing", [&, this]() {
        if (stamp_sec < last_scan_time) {
            LOG(WARNING) << "Time inconsistency detected in Lidar data stream";
            lidar_cache_.clear();
        }

        common::LidarScan lidar_scan;
        lidar_processing_->processing(msg, lidar_scan);
        lidar_cache_.push_back(lidar_scan);
        last_scan_time = stamp_sec;
    });

    last_scan_time = stamp_sec;
    return;
}

void RosInterface::imuCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    static sensor_msgs::msg::Imu last_imu_msg;
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);

    if (options::kRedundancy) {
        if (imu_msg->linear_acceleration.z == last_imu_msg.linear_acceleration.z &&
            imu_msg->angular_velocity.z == last_imu_msg.angular_velocity.z) {
            last_imu_msg = *imu_msg;
            return;
        }
    }

    double timestamp = stampToSec(imu_msg->header.stamp);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_imu_) {
            LOG(WARNING) << "Time inconsistency detected in Imu data stream";
            imu_cache_.clear();
        }

        imu_cache_.push_back(imu_msg);
        last_imu_msg = *imu_msg;
        last_timestamp_imu_ = timestamp;
    }
    return;
}

void RosInterface::kinematicImuCallBack(const unitree_legged_msgs::msg::HighState::ConstSharedPtr& msg) {
    static unitree_legged_msgs::msg::HighState last_highstate_msg;
    auto highstate_msg = std::make_shared<unitree_legged_msgs::msg::HighState>(*msg);

    if (options::kRedundancy) {
        if (highstate_msg->imu.accelerometer[2] == last_highstate_msg.imu.accelerometer[2] &&
            highstate_msg->imu.gyroscope[2] == last_highstate_msg.imu.gyroscope[2]) {
            last_highstate_msg = *highstate_msg;
            return;
        }
    }

    double timestamp = stampToSec(highstate_msg->stamp);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_kin_imu_) {
            LOG(WARNING) << "Time inconsistency detected in Kin. Imu data stream";
            kin_imu_cache_.clear();
        }

        common::KinImuMeas kin_imu_meas;

        kinematics_->processing(*highstate_msg, kin_imu_meas);

        kin_imu_cache_.push_back(kin_imu_meas);
        last_timestamp_kin_imu_ = timestamp;
        last_highstate_msg = *highstate_msg;
    }

    if (pub_joint_tf_enable_) {
        static std::vector<std::string> joint_names = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = last_highstate_msg.stamp;
        joint_state.name = joint_names;
        const auto& motor = last_highstate_msg.motor_state;
        joint_state.position = {
            motor[0].q, motor[1].q, motor[2].q, motor[3].q, motor[4].q,  motor[5].q,
            motor[6].q, motor[7].q, motor[8].q, motor[9].q, motor[10].q, motor[11].q,
        };
        joint_state.velocity = {
            motor[0].dq, motor[1].dq, motor[2].dq, motor[3].dq, motor[4].dq,  motor[5].dq,
            motor[6].dq, motor[7].dq, motor[8].dq, motor[9].dq, motor[10].dq, motor[11].dq,
        };
        pub_joint_state_->publish(joint_state);
    }
    return;
}

bool RosInterface::syncPackage() {
    static bool lidar_push_ = false;

    std::lock_guard<std::mutex> lk(mutex_);

    // pack lidar and  imu
    if (options::kImuUse) {
        if (lidar_cache_.empty() || imu_cache_.empty()) return false;

        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_) { return false; }

        double imu_time = stampToSec(imu_cache_.front()->header.stamp);
        measure_.imus_.clear();
        while ((!imu_cache_.empty()) && (imu_time < lidar_end_time_)) {
            imu_time = stampToSec(imu_cache_.front()->header.stamp);
            if (imu_time > lidar_end_time_) break;
            measure_.imus_.push_back(imu_cache_.front());
            imu_cache_.pop_front();
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    // pack lidar, kin. and  imu
    if (options::kKinAndImuUse) {
        if (lidar_cache_.empty() || kin_imu_cache_.empty()) return false;

        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_kin_imu_ < lidar_end_time_) { return false; }

        double kin_imu_time = kin_imu_cache_.front().time_stamp_;
        measure_.kin_imus_.clear();
        while ((!kin_imu_cache_.empty()) && (kin_imu_time < lidar_end_time_)) {
            kin_imu_time = kin_imu_cache_.front().time_stamp_;
            if (kin_imu_time > lidar_end_time_) break;
            measure_.kin_imus_.push_back(kin_imu_cache_.front());
            kin_imu_cache_.pop_front();
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    throw std::runtime_error("Error sync package");
    return false;
}

void RosInterface::publishOdomTFPath(double end_time) {
    // odometry
    odom_world_.header.stamp = secToStamp(end_time);
    odom_world_.pose.pose.position.x = kilo_->getPos()(0);
    odom_world_.pose.pose.position.y = kilo_->getPos()(1);
    odom_world_.pose.pose.position.z = kilo_->getPos()(2);
    q_eigen_ = Eigen::Quaterniond(kilo_->getRot());
    odom_world_.pose.pose.orientation.w = q_eigen_.w();
    odom_world_.pose.pose.orientation.x = q_eigen_.x();
    odom_world_.pose.pose.orientation.y = q_eigen_.y();
    odom_world_.pose.pose.orientation.z = q_eigen_.z();
    pub_odom_world_->publish(odom_world_);

    // tf
    tf_msg_.header.stamp = odom_world_.header.stamp;
    tf_msg_.transform.translation.x = odom_world_.pose.pose.position.x;
    tf_msg_.transform.translation.y = odom_world_.pose.pose.position.y;
    tf_msg_.transform.translation.z = odom_world_.pose.pose.position.z;
    tf_msg_.transform.rotation = odom_world_.pose.pose.orientation;
    if (tf_broadcaster_) { tf_broadcaster_->sendTransform(tf_msg_); }

    // path
    pose_path_.header.stamp = odom_world_.header.stamp;
    pose_path_.pose = odom_world_.pose.pose;
    path_world_.poses.push_back(pose_path_);
    pub_path_->publish(path_world_);
}

void RosInterface::publishPointcloudWorld(double end_time) {
    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(*cloud_down_world_, pcl_msg);
    pcl_msg.header.stamp = secToStamp(end_time);
    pcl_msg.header.frame_id = "camera_init";
    pub_pointcloud_world_->publish(pcl_msg);
}

void RosInterface::runReset() {
    cloud_raw_.reset(new PointCloudType());
    cloud_down_body_.reset(new PointCloudType());
    cloud_down_world_.reset(new PointCloudType());

    success_pts_size = 0;
}

void RosInterface::run() {
    if (!this->syncPackage()) return;
    this->runReset();

    cloud_raw_ = measure_.lidar_scan_.cloud_;
    double end_time = measure_.lidar_scan_.lidar_end_time_;
    if (!kilo_->process(measure_, cloud_down_body_, cloud_down_world_, success_pts_size)) {
        LOG(WARNING) << "KILO processing failed";
        return;
    }

    LOG(INFO) << "pcl raw size:  " << cloud_raw_->points.size()
              << "  pcl down size: " << cloud_down_body_->points.size();
    LOG(INFO) << "useful pcl percent :  " << 100 * ((double)(success_pts_size) / cloud_down_body_->points.size())
              << " %";

    this->publishOdomTFPath(end_time);
    this->publishPointcloudWorld(end_time);

    if (traj_saver_) { traj_saver_->write(end_time, kilo_->getRot(), kilo_->getPos()); }

    if (pcd_saver_) { pcd_saver_->save(cloud_down_world_); }

    return;
}

}  // namespace legkilo

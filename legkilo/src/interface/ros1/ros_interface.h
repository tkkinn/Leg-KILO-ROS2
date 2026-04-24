#ifndef LEG_KILO_ROS_INTERFACE_H
#define LEG_KILO_ROS_INTERFACE_H

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "common/eigen_types.hpp"
#include "common/pcd_saver.hpp"
#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
#include "common/trajectory_saver.hpp"
#include "interface/ros1/options.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <unitree_go/msg/low_state.hpp>

namespace legkilo {
class Kinematics;
class LidarProcessing;
class KILO;
}  // namespace legkilo

namespace legkilo {

class RosInterface {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RosInterface() = delete;
    explicit RosInterface(rclcpp::Node::SharedPtr node);
    ~RosInterface();

    void rosInit(const std::string& config_file);
    void run();

   private:
    bool initParamAndReset(const std::string& config_file);
    void subscribeLidar();
    void subscribeKinematicImu();
    void subscribeImu();
    void lidarCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void lidarCallBack(const livox_interfaces::msg::CustomMsg::ConstSharedPtr& msg);
    void imuCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void kinematicImuCallBack(const unitree_go::msg::LowState::ConstSharedPtr& msg);
    bool syncPackage();
    void runReset();
    void publishOdomTFPath(double end_time);
    void publishPointcloudWorld(double end_time);
    void publishPointcloudBody(double end_time);  // without undistort

    rclcpp::Node::SharedPtr node_;

    // subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_raw_;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_lidar_livox_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_kinematic_raw_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_world_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_world_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;

    nav_msgs::msg::Odometry odom_world_;
    nav_msgs::msg::Path path_world_;
    geometry_msgs::msg::TransformStamped tf_msg_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    Eigen::Quaterniond q_eigen_;
    geometry_msgs::msg::PoseStamped pose_path_;

    // module
    std::unique_ptr<LidarProcessing> lidar_processing_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<KILO> kilo_;
    std::unique_ptr<TrajectorySaver> traj_saver_;
    std::unique_ptr<PcdSaver> pcd_saver_;

    // meaure
    std::deque<common::LidarScan> lidar_cache_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_cache_;
    std::deque<common::KinImuMeas> kin_imu_cache_;
    common::MeasGroup measure_;

    // sync package
    std::mutex mutex_;
    double last_timestamp_lidar_ = 0.0;
    double last_timestamp_imu_ = -1.0;
    double last_timestamp_kin_imu_ = -1.0;
    double lidar_end_time_ = 0.0;
    bool lidar_pushed_ = false;

    // FAST-LIO style software time sync
    bool time_sync_en_ = false;
    bool timediff_set_flg_ = false;
    double time_diff_lidar_to_imu_ = 0.0;
    double timediff_lidar_wrt_imu_ = 0.0;

    // initialization
    double init_time_ = 0.1;
    bool init_flag_ = true;

    // pcl
    CloudPtr cloud_raw_;
    CloudPtr cloud_down_body_;
    CloudPtr cloud_down_world_;

    // LOG
    size_t success_pts_size = 0;

    // vis
    bool pub_joint_tf_enable_ = true;
};

}  // namespace legkilo
#endif  // LEG_KILO_ROS_INTERFACE_H

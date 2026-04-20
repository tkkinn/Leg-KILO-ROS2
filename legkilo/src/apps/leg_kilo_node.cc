#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "common/glog_utils.hpp"
#include "common/timer_utils.hpp"
#include "interface/ros1/ros_interface.h"

DEFINE_string(config_file, "config/leg_fusion.yaml", "Path to the YAML file");
void sigHandle(int sig) {
    legkilo::options::FLAG_EXIT.store(true);
    rclcpp::shutdown();
    LOG(INFO) << "catch sig " << sig << "  FLAG_EXIT = True";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    signal(SIGINT, sigHandle);

    std::unique_ptr<legkilo::Logging> logging(new legkilo::Logging(argc, argv, "logs"));
    auto node = std::make_shared<rclcpp::Node>("legkilo");
    std::unique_ptr<legkilo::RosInterface> ros_interface_node = std::make_unique<legkilo::RosInterface>(node);

    if (FLAGS_config_file.empty()) {
        LOG(ERROR) << "YAML configuration file path not provided. Use --config_path=<path>.";
        return -1;
    }

    ros_interface_node->rosInit(FLAGS_config_file);

    LOG(INFO) << "Leg KILO Node Starts";

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::WallRate rate(5000.0);
    while (rclcpp::ok() && !legkilo::options::FLAG_EXIT.load()) {
        executor.spin_some();
        ros_interface_node->run();
        rate.sleep();
    }
    legkilo::options::FLAG_EXIT.store(true);

    // Explicitly destroy ros_interface_node to ensure proper cleanup
    ros_interface_node.reset();
    LOG(INFO) << "RosInterface destroyed";
    LOG(INFO) << "Leg KILO Node Ends";
    legkilo::Timer::logAllAverTime();
    logging->flushLogFiles();
    rclcpp::shutdown();
    return 0;
}
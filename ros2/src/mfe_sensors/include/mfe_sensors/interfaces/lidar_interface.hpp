#ifndef LIDAR_INTERFACE
#define LIDAR_INTERFACE

#include "sensor_node.hpp"

#include <string>
#include <memory>

#include "fmt/core.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mfe_sensors {

class LidarInterface : public mfe_sensors::SensorNode
{
public:
    explicit LidarInterface(const std::string &camera_node_name, bool intra_process_comms = false) 
    : SensorNode(camera_node_name, intra_process_comms) { }

    virtual ~LidarInterface();

    std::string sensor_topic_name;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    std::unique_ptr<sensor_msgs::msg::PointCloud2> curr_pc;

private:

    virtual void publish() = 0;
    virtual void preprocess(const sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr) = 0;

};

}
#endif
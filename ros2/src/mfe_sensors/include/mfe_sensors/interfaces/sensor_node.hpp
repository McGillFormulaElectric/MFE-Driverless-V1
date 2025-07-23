#ifndef SENSOR_NODE
#define SENSOR_NODE
#include <string> 
#include <memory>

#include "fmt/core.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rclcpp/qos.hpp"

#include "std_msgs/msg/string.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace mfe_sensors {

class SensorNode: public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit SensorNode(const std::string &sensor_node_name, bool intra_process_comms = false) : 
    rclcpp_lifecycle::LifecycleNode(
        sensor_node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    ),
    sensor_name{std::move(sensor_node_name)},
    qos_{rclcpp::SensorDataQoS()},
    lifecycle_pub_{this->create_publisher<std_msgs::msg::String>(sensor_name, qos_)}
    { }

    virtual ~SensorNode() = default;

    // Define Lifecycle methods
    virtual CallbackReturn on_configure(
        const rclcpp_lifecycle::State &
    ) = 0;

    virtual CallbackReturn on_activate(
        const rclcpp_lifecycle::State &
    ) = 0;

    virtual CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &
    ) = 0;

    virtual CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &
    ) = 0;

protected:
    std::string sensor_name;
    const rclcpp::QoS qos_;

private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> lifecycle_pub_;
};

}
#endif 

#include <string> 
#include <memory>

#include "fmt/core.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"

namespace mfe_sensors {

class SensorNode: public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit SensorNode(const std::string &sensor_node_name, bool intra_process_comms = false) : 
    rclcpp_lifecycle::LifecycleNode(
        sensor_node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    ) { 
        this->sensor_name = sensor_node_name;
        this->lifecycle_pub_ = this->create_publisher<std_msgs::msg::String>(
            fmt::format("{}", sensor_node_name), 10
        );
    }

    virtual ~SensorNode() = default;

    // Define Lifecycle methods
    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &
    );

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &
    );

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &
    );

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &
    );
 
private:
    std::string sensor_name;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> lifecycle_pub_;
};

}
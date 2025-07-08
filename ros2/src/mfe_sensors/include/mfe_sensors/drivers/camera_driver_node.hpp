#include "base_driver_node.hpp"
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace mfe {

class CameraDriverNode : public BaseDriverNode<CameraDriverNode> {
public:
    CameraDriverNode() : BaseDriverNode("camera_driver_node") {}    // default constructs

    CallbackReturn onConfigure() {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/image_raw", rclcpp::QoS(10)
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn onActivate() {

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn onDeactivate() {

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn onCleanup() {
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn onShutdown() {

        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> image_pub_;
};

}   // mfe
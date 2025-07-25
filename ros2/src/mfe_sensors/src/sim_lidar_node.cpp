#include "mfe_sensors/interfaces/lidar_interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace mfe_sensors {

class SimLidarNode : public mfe_sensors::LidarInterface {
public:
    explicit SimLidarNode(const std::string &camera_node_name) : LidarInterface(camera_node_name)
    { }

    ~SimLidarNode() { }

private:

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Configuring...");
        this->curr_pc = std::make_unique<sensor_msgs::msg::PointCloud2>();

        this->declare_parameter<std::string>("sensor_topic_name", "/fsds/lidar/Lidar1");
        this->sensor_topic_name = this->get_parameter("sensor_topic_name").as_string();

        this->pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            fmt::format("{}/data", this->sensor_name),
            this->qos_
        );
        this->pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->sensor_topic_name,
            this->qos_,
            std::bind(&SimLidarNode::preprocess, this, std::placeholders::_1) 
        );
        RCLCPP_INFO(get_logger(), "Configured successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Activating node and subscriptions...");

        RCLCPP_INFO(get_logger(), "Subscription and node activated.");

        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Deactivating subscription...");

        this->pc_pub_.reset();
        this->pc_sub_.reset();

        RCLCPP_INFO(get_logger(), "Subscription deactivated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Cleaning up parameters and resources...");

        // Clean all non smart ptr fields
        this->sensor_name.clear();
        this->sensor_topic_name.clear();

        RCLCPP_INFO(get_logger(), "Shutting down...");
        return CallbackReturn::SUCCESS;
    }

    void publish() override {

    }

    void preprocess(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override {
        sensor_msgs::msg::PointCloud2 pc = *msg;
        this->pc_pub_->publish(pc);
    }
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mfe_sensors::SimLidarNode>("sim_lidar_node");
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());

  exe.spin();
  rclcpp::shutdown();
  return 0;
}
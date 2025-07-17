#include "mfe_sensors/include/mfe_sensors/drivers/lidar_driver_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

using asio::ip::udp;

namespace mfe_sensors
{

VelodyneLidar::VelodyneLidar(const rclcpp::NodeOptions & options)
: BaseDriverNode<VelodyneLidar>("velodyne_lidar", options),
  running_(false)
{
  declare_parameter("frame_id", "velodyne");
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn VelodyneLidar::on_configure()
{
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>
( "velodyne_points", rclcpp::SensorDataQoS());
 try {
    socket_ = std::make_unique<udp::socket>(
      *io_context_, udp::endpoint(udp::v4(), 2368));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open UDP socket: %s", e.what());
    return 1;
  }
    RCLCPP_INFO(get_logger(), "UDP socket successfully opened.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn VelodyneLidar::on_activate()
{
  publisher_->on_activate();
  running_ = true;
  RCLCPP_INFO(get_logger(), "Velodyne Activated.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn VelodyneLidar::on_cleanup()
{
  running_ = false;
  socket_.reset();
  io_context_.reset();
  publisher_.reset();

    RCLCPP_INFO(get_logger(), "UDP socket successfully opened");

  return CallbackReturn::SUCCESS;
}

void VelodyneLidar::start_udp_listener()
{
  auto buffer = std::make_shared<std::vector<uint8_t>>(1206);  // VLP-16 packet size

  socket_->async_receive(
    asio::buffer(*buffer),
    [this, buffer](const asio::error_code & ec, std::size_t bytes_recvd) {
      if (!ec && bytes_recvd == 1206) {
        handle_packet(*buffer);
      } else {
        RCLCPP_WARN(this->get_logger(), "UDP receive error: %s", ec.message().c_str());
      }

      if (running_) {
        start_udp_listener();
      }
    });
}

void VelodyneLidar::handle_packet(const std::vector<uint8_t> & data)
{
  // TODO: Properly parse Velodyne data format
  auto msg = sensor_msgs::msg::PointCloud2();
  msg.header.stamp = now();
  msg.header.frame_id = this->get_parameter("frame_id").as_string();

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz", "intensity");
  modifier.resize(0);  // Will resize as needed after parsing

  publisher_->publish(std::move(msg));
}

bool VelodyneLidar::publish_raw_data()
{
  // Optional — used if you batch/aggregate full scans before publishing
  return true;
}

}
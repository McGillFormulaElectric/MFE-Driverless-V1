<<<<<<< HEAD
#include "mfe_sensors/interfaces/sensor_node.hpp"
#include "mfe_sensors/interfaces/lidar_interface.hpp"
#include "mfe_sensors/drivers/base_driver_node.hpp"
=======
#include "mfe_sensors/interfaces/lidar_interface.hpp"

namespace mfe_sensors {

LidarInterface::~LidarInterface() = default;
>>>>>>> 3fc9d500b36d5a3fd2c0165a7dbf7af94276480c

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <asio.hpp>
#include <memory>
#include <thread>

namespace mfe_sensors
{

class VelodyneLidar : public BaseDriverNode<VelodyneLidar>, public BaseLidarNode
{
public:
  explicit VelodyneLidar(const rclcpp::NodeOptions & options);

  int on_configure();
  int on_activate();
  int on_cleanup();
  bool publish_raw_data();

private:
  void start_udp_listener();
  void handle_packet(const std::vector<uint8_t> & data);

  std::unique_ptr<asio::io_context> io_context_;
  std::unique_ptr<asio::ip::udp::socket> socket_;
  std::thread io_thread_;
  bool running_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}
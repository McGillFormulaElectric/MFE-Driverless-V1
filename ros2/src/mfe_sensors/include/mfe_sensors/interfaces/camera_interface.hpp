#ifndef CAMERA_INTERFACE
#define CAMERA_INTERFACE

#include "sensor_node.hpp"

#include <string>
#include <memory>

#include "fmt/core.h"

#include "sensor_msgs/msg/image.hpp"

namespace mfe_sensors {

class CameraInterface : public mfe_sensors::SensorNode
{
public:
    explicit CameraInterface(const std::string &camera_node_name) : SensorNode(camera_node_name, true) { }

    virtual ~CameraInterface() = default;

    std::unique_ptr<sensor_msgs::msg::Image> curr_img; 

private:

    virtual void capture_and_publish() = 0;

};

}
#endif
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace mfe_simulation {
    class FSDSNode : public rclcpp::Node {
        public:
            FSDSNode(const rclcpp::NodeOptions &options);
    };

}   // mfe_simulation
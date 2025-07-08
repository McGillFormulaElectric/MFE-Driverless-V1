#ifndef BASE_DRIVER_NODE_HPP_
#define BASE_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace mfe {

// CRTP base: Derived must implement onConfigure(), onActivate(), onDeactivate(), onCleanup(), onShutdown()
template <typename Derived>
class BaseDriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit BaseDriverNode(const std::string &node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
  {
    // bind lifecycle callbacks
    this->register_on_configure(
      std::bind(&BaseDriverNode::handle_configure, this, std::placeholders::_1));
    this->register_on_activate(
      std::bind(&BaseDriverNode::handle_activate, this, std::placeholders::_1));
    this->register_on_deactivate(
      std::bind(&BaseDriverNode::handle_deactivate, this, std::placeholders::_1));
    this->register_on_cleanup(
      std::bind(&BaseDriverNode::handle_cleanup, this, std::placeholders::_1));
    this->register_on_shutdown(
      std::bind(&BaseDriverNode::handle_shutdown, this, std::placeholders::_1));
  }

protected:
  // Generic handlers delegate to Derived
  CallbackReturn handle_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s: Configuring...");
    return static_cast<Derived *>(this)->onConfigure();
  }
  CallbackReturn handle_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s: Activating...");
    return static_cast<Derived *>(this)->onActivate();
  }
  CallbackReturn handle_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s: Deactivating...");
    return static_cast<Derived *>(this)->onDeactivate();
  }
  CallbackReturn handle_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s: Cleaning up...");
    return static_cast<Derived *>(this)->onCleanup();
  }
  CallbackReturn handle_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s: Shutting down...");
    return static_cast<Derived *>(this)->onShutdown();
  }
};

}

#endif  // BASE_DRIVER_NODE_HPP_
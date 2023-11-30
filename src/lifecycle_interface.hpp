/**
 * @file
 * @brief This file defines the AoxLifecycleNodeInterface class, an interface for managing ROS 2 nodes.
 */

#ifndef LIFECYCLE_INTERFACE_HPP_
#define LIFECYCLE_INTERFACE_HPP_
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "aox_lifecycle_msg/msg/transition.hpp"
#include "aox_lifecycle_msg/srv/change_state.hpp"
#include "aox_lifecycle_msg/msg/state.hpp"
#include <iostream>

/// Interface class for a managed node.
/**
 * If the callback function returns successfully,
 * the specified transition is completed.
 * If the callback function fails or throws an
 * uncaught exception, the on_error function is
 * called.
 * By default, all functions remain optional to overwrite
 * and return true. Except the on_error function, which
 * returns false and thus goes to shutdown state.
 */

class AoxLifecycleNodeInterface
{
protected:
  // //RCLCPP_LIFECYCLE_PUBLIC
  // AoxLifecycleNodeInterface() {}

public:
  /**
   * @brief Enumeration representing the return status of callback functions.
   */
  enum class CallbackReturn : uint8_t
  {
    SUCCESS, ///< Indicates a successful callback execution.
    FAILURE, ///< Indicates a failure in the callback execution.
    ERROR    ///< Indicates an error condition during the callback execution.
  };

public:
  /**
   * @brief Constructor for the AoxLifecycleNodeInterface class.
   * @param node_name The name of the node.
   */
  explicit AoxLifecycleNodeInterface(const std::string &node_name)
  {
  }

  /**
   * @brief Virtual destructor for the AoxLifecycleNodeInterface class.
   */
  virtual ~AoxLifecycleNodeInterface() = default;
  /**
   * @brief Callback function for the configure transition.
   * @return CallbackReturn::SUCCESS if successful, CallbackReturn::FAILURE on failure, CallbackReturn::ERROR on error.
   */

  virtual CallbackReturn
  on_configure() = 0;

  /**
   * @brief Callback function for the shutdown transition.
   * @return CallbackReturn::SUCCESS if successful, CallbackReturn::FAILURE on failure, CallbackReturn::ERROR on error.
   */
  virtual CallbackReturn
  on_shutdown() = 0;

  /**
   * @brief Callback function for the activate transition.
   * @return CallbackReturn::SUCCESS if successful, CallbackReturn::FAILURE on failure, CallbackReturn::ERROR on error.
   */
  virtual CallbackReturn
  on_activate() = 0;

  /**
   * @brief Callback function for the deactivate transition.
   * @return CallbackReturn::SUCCESS if successful, CallbackReturn::FAILURE on failure, CallbackReturn::ERROR on error.
   */
  virtual CallbackReturn
  on_deactivate() = 0;

  /**
   * @brief Callback function for the erroneous transition.
   * @return CallbackReturn::ERROR always to indicate an error condition.
   */
  virtual CallbackReturn
  on_error() = 0;

  // RCLCPP_LIFECYCLE_PUBLIC
};

#endif

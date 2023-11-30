#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "/home/devuser/AOX/ROS2/src/lcm/src/lifecycle_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aox_lifecycle_msg/srv/get_state.hpp"
#include "aox_lifecycle_msg/msg/state.hpp"
#include "aox_lifecycle_msg/msg/transition.hpp"

using namespace std::chrono_literals;

/**
 * @brief Represents a managed ROS 2 node with lifecycle callbacks.
 *
 * This class inherits from AoxLifecycleNodeInterface and rclcpp::Node,
 * providing implementations for lifecycle callbacks and additional functionality.
 * The four main States of this Node are:
 * 1. PRIMARY_STATE_UNCONFIGURED
 * 2. PRIMARY_STATE_ACTIVE
 * 3. PRIMARY_STATE_INACTIVE
 * 4. PRIMARY_STATE_SHUTDOWN
 * The states of this node are controlled by an LCM master
 */
class MyManagedNode : public AoxLifecycleNodeInterface, public rclcpp::Node
{
public:
  /**
   * @brief Constructor for MyManagedNode.
   * @param node_name The name of the node.
   */
  explicit MyManagedNode(const std::string &node_name) : AoxLifecycleNodeInterface(node_name), Node(node_name)
  {
    currentState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_UNCONFIGURED;
  }
  /**
   * @brief Callback function for configure transition.
   * Transition on CallbackReturn=SUCCESS : PRIMARY_STATE_UNCONFIGURED->PRIMARY_STATE_INACTIVE
   * @return CallbackReturn value indicating the result of the callback.
   * @see AoxLifecycleNodeInterface::on_configure
   */
  CallbackReturn on_configure() override
  {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node configured");
    currentState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE;
    return CallbackReturn::SUCCESS;
    // Your additional configuration code here
  }
  /**
   * @brief Callback function for the activate transition.
   * Transition on CallbackReturn=SUCCESS : PRIMARY_STATE_INACTIVE->PRIMARY_STATE_ACTIVE
   * @return CallbackReturn value indicating the result of the callback.
   * @see AoxLifecycleNodeInterface::on_activate
   */
  CallbackReturn on_activate() override
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node activated");
    currentState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE;
    // Starting a publisher in Active state
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
    timer_ = this->create_wall_timer(
        1s, std::bind(&MyManagedNode::publishMessage, this));
    return CallbackReturn::SUCCESS;
  }
  /**
   * @brief Callback function for the deactivate transition.
   * Transition on CallbackReturn=SUCCESS : PRIMARY_STATE_ACTIVE->PRIMARY_STATE_INACTIVE
   * @return CallbackReturn value indicating the result of the callback.
   * @see AoxLifecycleNodeInterface::on_deactivate
   */
  CallbackReturn on_deactivate() override
  {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node deactivated");
    this->timer_->cancel();
    currentState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE;
    return CallbackReturn::SUCCESS;
  }
  /**
   * @brief Callback function for the deactivate transition.
   * Possible Transitions on CallbackReturn=SUCCESS :
   * 1. PRIMARY_STATE_UNCONFIGURED-> PRIMARY_STATE_SHUTDOWN
   * 2. PRIMARY_STATE_ACTIVE-> PRIMARY_STATE_SHUTDOWN
   * 3. PRIMARY_STATE_INACTIVE-> PRIMARY_STATE_SHUTDOWN
   * @return CallbackReturn value indicating the result of the callback.
   * @see AoxLifecycleNodeInterface::on_deactivate
   */
  CallbackReturn on_shutdown() override
  {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node shut down");
    currentState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
    rclcpp::shutdown();
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Callback function for the erroneous transition.
   * @return CallbackReturn::ERROR always to indicate an error condition.
   */
  CallbackReturn on_error() override
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node error!!");
    return CallbackReturn::ERROR;
  }
  /**
   * @brief Callback function for the change_state service.
   * This function will be called whenever a client tries to use the "change_state" service
   * "change_state" service is used to change the states of this node
   * @param request Request of ChangeState interface
   * @param response Response of ChangeState interface
   * @return void
   */
  void change_state(const std::shared_ptr<aox_lifecycle_msg::srv::ChangeState::Request> request,
                    std::shared_ptr<aox_lifecycle_msg::srv::ChangeState::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld",
                request->transition.id);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint newState = request->transition.id;
    switch (request->transition.id)
    {
    case aox_lifecycle_msg::msg::Transition::TRANSITION_CONFIGURE:
      newState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE;
      break;
    case aox_lifecycle_msg::msg::Transition::TRANSITION_ACTIVATE:
      newState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE;
      break;
    case aox_lifecycle_msg::msg::Transition::TRANSITION_DEACTIVATE:
      newState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE;
      break;
    case aox_lifecycle_msg::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
      newState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
      break;
    case aox_lifecycle_msg::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN:
      newState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
      break;
    case aox_lifecycle_msg::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN:
      newState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
      break;
    }
    // If Transition is valid, call corrensponding Callback function
    if (isValidTransition(newState))
    {

      if (request->transition.id == aox_lifecycle_msg::msg::Transition::TRANSITION_CONFIGURE)
      {
        response->success = true;
        this->on_configure();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transition success!!: [%ld]", (long int)response->success);
      }
      else if (request->transition.id == aox_lifecycle_msg::msg::Transition::TRANSITION_ACTIVATE)
      {
        response->success = true;
        this->on_activate();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transition success!!: [%ld]", (long int)response->success);
      }
      else if (request->transition.id == aox_lifecycle_msg::msg::Transition::TRANSITION_DEACTIVATE)
      {
        response->success = true;
        this->on_deactivate();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transition success!!: [%ld]", (long int)response->success);
      }

      else if ((request->transition.id == aox_lifecycle_msg::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN) || (request->transition.id == aox_lifecycle_msg::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN) || (request->transition.id == aox_lifecycle_msg::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN))
      {
        response->success = true;
        this->on_shutdown();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transition success!!: [%ld]", (long int)response->success);
      }
    }
    else
    {
      response->success = false;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid transition!!: [%ld]", (long int)response->success);
    }
  }
  /**
   * @brief Callback function for the get_state service.
   * "get_state" service is used to get the states of this node
   * @param request Request of GetState interface
   * @param response Response of GetState interface
   * @return void
   */
  void get_state(const std::shared_ptr<aox_lifecycle_msg::srv::GetState::Request> request,
                 std::shared_ptr<aox_lifecycle_msg::srv::GetState::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request fot get state");
    response->current_state.id = this->currentState;
  }

private:
  uint currentState;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief Fuction for checking whether the transition is valid
   * @param newState New state to which state machine will transition to
   * @return TRUE if the transition is valid, else FALSE
   */
  bool isValidTransition(uint newState)
  {
    switch (currentState)
    {
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_UNCONFIGURED:
    {
      // Allow transitions to Active to Inactive or Shutdown
      bool res = newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE || newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RES: [%ld]", res);
      return newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE || newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
    }
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE:
      // Allow transitions to Active to Inactive or Shutdown
      return newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE || newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;

    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE:
      // Allow transitions to Inactive to Active or Shutdown
      return newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE || newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;

    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN:
      // Once in the Shutdown state, no further transitions are allowed
      return false;
    }

    return false; // Default case: disallow unknown transitions
  }
  /**
   * @brief Callback function for the timer to publish a message.
   */
  void publishMessage()
  {
    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);
    RCLCPP_INFO(get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
    pub_->publish(std::move(msg));
  }
};
/**
 * @brief Main function to initialize and run the managed ROS 2 node.
 * @param argc Number of command-line arguments.
 * @param argv Command-line argument values.
 * @return 0 on successful execution, other values on failure.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto managed_node = std::make_shared<MyManagedNode>("light_source_controller");
  auto change_state_callback = std::bind(&MyManagedNode::change_state, managed_node,
                                         std::placeholders::_1, std::placeholders::_2);
  rclcpp::Service<aox_lifecycle_msg::srv::ChangeState>::SharedPtr change_state_service =
      managed_node->rclcpp::Node::create_service<aox_lifecycle_msg::srv::ChangeState>("change_state", change_state_callback);
  auto get_state_callback = std::bind(&MyManagedNode::get_state, managed_node,
                                      std::placeholders::_1, std::placeholders::_2);
  rclcpp::Service<aox_lifecycle_msg::srv::GetState>::SharedPtr get_state_service =
      managed_node->rclcpp::Node::create_service<aox_lifecycle_msg::srv::GetState>("get_state", get_state_callback);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting up node...");
  rclcpp::spin(managed_node);

  rclcpp::shutdown();
  return 0;
}

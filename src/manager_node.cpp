
/**
 * @file
 * @brief This is LCM MASTER node
 *
 * Detailed description (if necessary).
 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_interface.hpp"
#include "aox_lifecycle_msg/srv/get_state.hpp"
#include "aox_lifecycle_msg/msg/state.hpp"
#include "aox_lifecycle_msg/msg/lcm_transition.hpp"
#include "aox_lifecycle_msg/srv/lcm_change_system_state.hpp"
#include "aox_lifecycle_msg/msg/lcm_system_state.hpp"
constexpr char const *node_change_state_topic = "change_state";
enum class CallbackReturn : uint8_t
{
  SUCCESS,
  FAILURE,
  ERROR,
};
/**
 * @brief Callback function for handling the response of lcm_get_state service.

 * @return void.
 */
void get_state_req_callback(const rclcpp::Client<aox_lifecycle_msg::srv::GetState>::SharedFuture future)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got the State change response!!!");
  auto result = future.get();
  switch (result->current_state.id)
  {
  case 0:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current state: PRIMARY_STATE_UNCONFIGURE");
    break;
  case 1:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current state: PRIMARY_STATE_ACTIVE");
    break;
  case 2:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current state: PRIMARY_STATE_INACTIVE");
    break;
  case 3:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current state: PRIMARY_STATE_SHUTDOWN");
    break;
  default:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Unknown state!!");
    break;
  }
}
/**
 * @brief Callback function for handling the response of lcm_change_state service.

 * @return void.
 */
void change_state_req_callback(const rclcpp::Client<aox_lifecycle_msg::srv::ChangeState>::SharedFuture future)
{
  auto result = future.get();

  if (result->success == true)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State change successfull!");
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State change failed!");
  }
}

/**
 * @brief Represents a LCM master node with lifecycle callbacks.
 * LCM master controls the state of Light source controller
 * This class inherits from AoxLifecycleNodeInterface and rclcpp::Node,
 * providing implementations for lifecycle callbacks and additional functionality.
 * The four main States of this Node are:
 * 1. PRIMARY_STATE_UNCONFIGURED
 * 2. PRIMARY_STATE_ACTIVE
 * 3. PRIMARY_STATE_INACTIVE
 * 4. PRIMARY_STATE_SHUTDOWN
 * This node also has 5 system states in "PRIMARY_STATE_ACTIVE" State:
 * 1.SYSTEMSTATE_ACTIVE_INITIALIZATION
 * 2.SYSTEM_STATE_ACTIVE_PRE_SURGERY
 * 3.SYSTEM_STATE_ACTIVE_SURGERY
 * 4.SYSTEM_STATE_ACTIVE_POST_SURGERY
 * 5.SYSTEM_STATE_ACTIVE_UPDATE
 *  When the system state of the system in "SYSTEM_STATE_ACTIVE_SURGERY",Light source controller will be active
 */
class ManagerNode : public AoxLifecycleNodeInterface, public rclcpp::Node
{
public:
  ManagerNode(const std::string &node_name)
      : AoxLifecycleNodeInterface(node_name), Node(node_name)
  {
    client_change_state = this->create_client<aox_lifecycle_msg::srv::ChangeState>("change_state");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LCM master is active....!!!");
    LcmCurrentMainState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_UNCONFIGURED;
    LcmSystemState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_INITIALIZATION;
  }
  /**
   * @brief Callback function for configure transition.
   * Transition on CallbackReturn=SUCCESS : PRIMARY_STATE_UNCONFIGURED->PRIMARY_STATE_INACTIVE
   * @return CallbackReturn value indicating the result of the callback.
   * @see AoxLifecycleNodeInterface::on_configure
   */
  CallbackReturn on_configure() override
  {
    // Handle behavior during Initialization state transition
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node in PRIMARY_STATE_INACTIVE state");
    LcmCurrentMainState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE;
    return CallbackReturn::SUCCESS;
  }
  /**
   * @brief Callback function for the activate transition.
   * Transition on CallbackReturn=SUCCESS : PRIMARY_STATE_INACTIVE->PRIMARY_STATE_ACTIVE
   * @return CallbackReturn value indicating the result of the callback.
   * @see AoxLifecycleNodeInterface::on_activate
   */
  CallbackReturn on_activate() override
  {
    // Handle behavior during Active state transition
    auto request_change_state = std::make_shared<aox_lifecycle_msg::srv::ChangeState::Request>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node in PRIMARY_STATE_ACTIVE state");
    LcmCurrentMainState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE;
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
    // Handle behavior during Active state transition
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node in PRIMARY_STATE_INACTIVE state");
    LcmCurrentMainState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE;
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
    // Handle behavior during Shutdown state transition
    auto request_change_state = std::make_shared<aox_lifecycle_msg::srv::ChangeState::Request>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node in PRIMARY_STATE_SHUTDOWN state");
    switch (LcmCurrentMainState)
    {
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_UNCONFIGURED:
      request_change_state->transition.id = aox_lifecycle_msg::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
      break;
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE:
      request_change_state->transition.id = aox_lifecycle_msg::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
      break;
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE:
      request_change_state->transition.id = aox_lifecycle_msg::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
      break;

    default:
      break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Outgoing request\na: %ld",
                request_change_state->transition.id);
    auto result_change_state = client_change_state->async_send_request(request_change_state);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending a request to change state of the managed node to PRIMARY_STATE_SHUTDOWN..");
    LcmCurrentMainState = aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_change_state) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown completed!");
      rclcpp::shutdown();
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Shutdown failed!");
    }

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
   * @brief This function handles the system state transition. Callback function for the update transition.
   * @return CallbackReturn::ERROR always to indicate an error condition.
   */
  CallbackReturn on_update()
  {
    // Handle behavior during Update state transition
    // If this transition is successful, the system state of the node will change to SYSTEM_STATE_ACTIVE_UPDATE
    auto request_change_state = std::make_shared<aox_lifecycle_msg::srv::ChangeState::Request>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Feature yet to be implemented");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "System state of node:  SYSTEM_STATE_ACTIVE_UPDATE state");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Main state of node:  PRIMARY_STATE_ACTIVE state");
    LcmSystemState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_UPDATE;
    return CallbackReturn::SUCCESS;
  }
  /**
   * @brief This function handles the system state transition. Callback function for the presurgery transition.
   Transition on CallbackReturn=SUCCESS :SYSTEM_STATE_ACTIVE_INITIALIZATION->SYSTEM_STATE_ACTIVE_PRE_SURGERY
   * @return CallbackReturn::ERROR always to indicate an error condition.
   */
  CallbackReturn on_presurgery()
  {
    // At this transition, the state of the light source will change from PRIMARY_STATE_UNCONFIGURED->PRIMARY_STATE_INACTIVE
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "System state of node:  SYSTEM_STATE_ACTIVE_PRE_SURGERY state");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Main state of node:  PRIMARY_STATE_ACTIVE state");
    auto request_change_state = std::make_shared<aox_lifecycle_msg::srv::ChangeState::Request>();
    request_change_state->transition.id = aox_lifecycle_msg::msg::Transition::TRANSITION_CONFIGURE;
    auto result_change_state = client_change_state->async_send_request(request_change_state, change_state_req_callback);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending a request to change state of the managed node to PRIMARY_STATE_INACTIVE..");
    LcmSystemState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_PRE_SURGERY;
    return CallbackReturn::SUCCESS;
  }
  /**
   * @brief This function handles the system state transition. Callback function for the surgery transition.
   Transition on CallbackReturn=SUCCESS :SYSTEM_STATE_ACTIVE_PRE_SURGERY->SYSTEM_STATE_ACTIVE_SURGERY
   * @return CallbackReturn::ERROR always to indicate an error condition.
   */
  CallbackReturn on_surgery()
  {
    // At this transition, the state of the light source will change from PRIMARY_STATE_INACTIVE->PRIMARY_STATE_ACTIVE
    auto request_change_state = std::make_shared<aox_lifecycle_msg::srv::ChangeState::Request>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "System state of node:  SYSTEM_STATE_ACTIVE_SURGERY state");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Main state of node:  PRIMARY_STATE_ACTIVE state");
    request_change_state->transition.id = aox_lifecycle_msg::msg::Transition::TRANSITION_ACTIVATE;
    auto result_change_state = client_change_state->async_send_request(request_change_state, change_state_req_callback);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending a request to change state of the managed node to PRIMARY_STATE_ACTIVE..");
    LcmSystemState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_SURGERY;
    return CallbackReturn::SUCCESS;
  }
  /**
 * @brief This function handles the system state transition. Callback function for the postsurgery transition.
 Transition on CallbackReturn=SUCCESS :SYSTEM_STATE_ACTIVE_SURGERY->SYSTEM_STATE_ACTIVE_POST_SURGERY
 * @return CallbackReturn::ERROR always to indicate an error condition.
 */
  CallbackReturn on_postsurgery()
  {
    // At this transition, the state of the light source will change from PRIMARY_STATE_ACTIVE->PRIMARY_STATE_INACTIVE
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "System state of node:  SYSTEM_STATE_ACTIVE_POST_SURGERY state");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Main state of node:  PRIMARY_STATE_ACTIVE state");
    auto request_change_state = std::make_shared<aox_lifecycle_msg::srv::ChangeState::Request>();
    request_change_state->transition.id = aox_lifecycle_msg::msg::Transition::TRANSITION_DEACTIVATE;
    auto result_change_state = client_change_state->async_send_request(request_change_state, change_state_req_callback);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending a request to change state of the managed node to PRIMARY_STATE_INACTIVE..");
    LcmSystemState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_POST_SURGERY;
    return CallbackReturn::SUCCESS;
  }
  /**
* @brief This function handles the system state transition. Callback function for the cleanup transition.
Transition on CallbackReturn=SUCCESS :SYSTEM_STATE_ACTIVE_SURGERY->SYSTEM_STATE_ACTIVE_POST_SURGERY
* @return CallbackReturn::ERROR always to indicate an error condition.
*/
  CallbackReturn on_cleanup()
  {
    // Handle behavior during cleanup state transition
    // If this transition is successful, the system state of the node will change to SYSTEM_STATE_ACTIVE_INITIALIZATION
    auto request_change_state = std::make_shared<aox_lifecycle_msg::srv::ChangeState::Request>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Feature yet to be implemented");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "System state of node:  SYSTEM_STATE_ACTIVE_INITIALIZATION state");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Main state of node:  PRIMARY_STATE_ACTIVE state");
    LcmSystemState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_INITIALIZATION;
    return CallbackReturn::SUCCESS;
  }

  void run()
  {
  }
  /**
   * @brief Callback function for the lcm_change_state service.
   * This function will be called whenever a client tries to use the "lcm_change_state" service
   * "lcm_change_state" service is used to change the states of this node
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
   * @brief Callback function for the lcm_change_system_state service.
   * This function will be called whenever a client tries to use the "lcm_change_system_state" service
   * "lcm_change_system_state" service is used to change the states of this node
   * @param request Request of LcmChangeSystemState interface
   * @param response Response of LcmChangeSystemState interface
   * @return void
   */
  void change_system_state(const std::shared_ptr<aox_lifecycle_msg::srv::LcmChangeSystemState::Request> request,
                           std::shared_ptr<aox_lifecycle_msg::srv::LcmChangeSystemState::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld",
                request->transition.id);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint newState = request->transition.id;
    switch (request->transition.id)
    {
    case aox_lifecycle_msg::msg::LcmTransition::TRANSITION_PRESURGERY:
      newState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_PRE_SURGERY;
      break;
    case aox_lifecycle_msg::msg::LcmTransition::TRANSITION_SURGERY:
      newState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_SURGERY;
      break;
    case aox_lifecycle_msg::msg::LcmTransition::TRANSITION_POSTSURGERY:
      newState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_POST_SURGERY;
      break;
    case aox_lifecycle_msg::msg::LcmTransition::TRANSITION_INITIALIZATION:
      newState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_INITIALIZATION;
      break;
    case aox_lifecycle_msg::msg::LcmTransition::TRANSITION_UPDATE:
      newState = aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_UPDATE;
      break;
    }
    // If Transition is valid, call corrensponding Callback function
    if (isValidSystemTransition(newState))
    {

      if (request->transition.id == aox_lifecycle_msg::msg::LcmTransition::TRANSITION_PRESURGERY)
      {
        response->success = true;
        this->on_presurgery();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transition success!!: [%ld]", (long int)response->success);
      }
      else if (request->transition.id == aox_lifecycle_msg::msg::LcmTransition::TRANSITION_SURGERY)
      {
        response->success = true;
        this->on_surgery();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transition success!!: [%ld]", (long int)response->success);
      }
      else if (request->transition.id == aox_lifecycle_msg::msg::LcmTransition::TRANSITION_POSTSURGERY)
      {
        response->success = true;
        this->on_postsurgery();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transition success!!: [%ld]", (long int)response->success);
      }

      else if (request->transition.id == aox_lifecycle_msg::msg::LcmTransition::TRANSITION_INITIALIZATION)
      {
        response->success = true;
        this->on_cleanup();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transition success!!: [%ld]", (long int)response->success);
      }
      else if (request->transition.id == aox_lifecycle_msg::msg::LcmTransition::TRANSITION_UPDATE)
      {
        response->success = true;
        this->on_update();
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
   * @brief Callback function for the lcm_get_state service.
   * "lcm_get_state" service is used to get the states of this node
   * @param request Request of GetState interface
   * @param response Response of GetState interface
   * @return void
   */
  void get_state(const std::shared_ptr<aox_lifecycle_msg::srv::GetState::Request> request,
                 std::shared_ptr<aox_lifecycle_msg::srv::GetState::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request fot get state");
    response->current_state.id = this->LcmCurrentMainState;
  }

private:
  rclcpp::Client<aox_lifecycle_msg::srv::ChangeState>::SharedPtr client_change_state;
  rclcpp::Client<aox_lifecycle_msg::srv::GetState>::SharedPtr client_get_state;
  uint LcmCurrentMainState;
  uint LcmSystemState;
  /**
   * @brief Function for checking whether the transition is valid
   * @param newState New state to which state machine will transition to
   * @return TRUE if the transition is valid, else FALSE
   */
  bool isValidTransition(uint newState)
  {
    switch (LcmCurrentMainState)
    {
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_UNCONFIGURED:
      return newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE || newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
      break;
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE:
      return newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE || newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
      break;
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE:
    {
      if (newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE || newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN)
      {
        if (LcmSystemState == aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_INITIALIZATION)
        {
          return true;
        }
        else
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The SystemState of active state is not in Initialization state. Invalid transition!");
          return false;
        }
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid transition in Active State!");
        return false;
      }
      // return newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_INACTIVE || newState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN;
      break;
    }
    // No transitions from Shutdown state
    case aox_lifecycle_msg::msg::State::PRIMARY_STATE_SHUTDOWN:
      return false;
      break;
    default:
      return false;
    }
  }
  /**
   * @brief Function for checking whether the system transition is valid
   * @param newState New sysstem state to which state machine will transition to
   * @return TRUE if the transition is valid, else FALSE
   */
  bool isValidSystemTransition(uint newState)
  {
    if (LcmCurrentMainState == aox_lifecycle_msg::msg::State::PRIMARY_STATE_ACTIVE)
    {
      switch (LcmSystemState)
      {
      case aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_INITIALIZATION:
        return newState == aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_PRE_SURGERY || newState == aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_UPDATE;
        break;
      case aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_PRE_SURGERY:
        return newState == aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_SURGERY;
        break;
      case aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_SURGERY:
        return newState == aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_POST_SURGERY;
        break;
      case aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_POST_SURGERY:
        return newState == aox_lifecycle_msg::msg::LcmSystemState::SYSTEM_STATE_ACTIVE_INITIALIZATION;
        break;
      default:
        return false;
      }
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The System is not in ACTIVE state. Invalid transition!");
      return false;
    }
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

  auto manager_node = std::make_shared<ManagerNode>("lifecycle_master");
  // manager_node->run();
  auto change_state_callback = std::bind(&ManagerNode::change_state, manager_node,
                                         std::placeholders::_1, std::placeholders::_2);
  rclcpp::Service<aox_lifecycle_msg::srv::ChangeState>::SharedPtr lcm_change_state_service =
      manager_node->rclcpp::Node::create_service<aox_lifecycle_msg::srv::ChangeState>("lcm_change_state", change_state_callback);

  auto get_state_callback = std::bind(&ManagerNode::get_state, manager_node,
                                      std::placeholders::_1, std::placeholders::_2);
  rclcpp::Service<aox_lifecycle_msg::srv::GetState>::SharedPtr lcm_get_state_service =
      manager_node->rclcpp::Node::create_service<aox_lifecycle_msg::srv::GetState>("lcm_get_state", get_state_callback);

  auto change_system_state_callback = std::bind(&ManagerNode::change_system_state, manager_node,
                                                std::placeholders::_1, std::placeholders::_2);
  rclcpp::Service<aox_lifecycle_msg::srv::LcmChangeSystemState>::SharedPtr lcm_change_system_state_service =
      manager_node->rclcpp::Node::create_service<aox_lifecycle_msg::srv::LcmChangeSystemState>("lcm_change_system_state", change_system_state_callback);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting up node...");
  rclcpp::spin(manager_node);

  rclcpp::shutdown();
  return 0;
}

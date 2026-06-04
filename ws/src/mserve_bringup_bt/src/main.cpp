#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

// ==============================================================================
// Main entry point for bringup_bt.
// ==============================================================================

namespace lifecyclemanager_bt {

// ==============================================================================
// BT Action Node
// ==============================================================================

  class IsInState : public BT::ConditionNode {
  public:
    IsInState(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
          : BT::ConditionNode(name, config), node_(node)  {
      // Get key/value pair from input_ports
      auto it = config.input_ports.find("node_name");
      if (it == config.input_ports.end()) {
        throw std::runtime_error("IsInState: required input port 'node_name' not found");
      }
    
      // Extract the node_name from value
      node_name_ = it->second;
      
      client_ = node_->create_client<lifecycle_msgs::srv::GetState>("/" + node_name_ + "/get_state");
      
      RCLCPP_INFO(node_->get_logger(), "Client created for: %s", node_name_.c_str());
    }
    
      static BT::PortsList providedPorts(){
      return {
        BT::InputPort<std::string>("node_name"),
        BT::InputPort<std::string>("state")
      };
    }

    BT::NodeStatus tick() override {
      // Get the state name from input port
      auto state_res = getInput<std::string>("state");
      if (!state_res) {
        RCLCPP_ERROR(node_->get_logger(), "Missing input port [state]: %s", state_res.error().c_str());
        return BT::NodeStatus::FAILURE;
      }

      // Map state name to state ID 
      const auto& state_name = state_res.value();

      // Create the service request
      auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      
      // Call the service and wait for the result
      if (!client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available: /%s/get_state", node_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // Send the request and spin until the response is received
      auto future = client_->async_send_request(request);
      auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
      if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Service call timed out: /%s/get_state", node_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      // Check if the service call was successful
      const auto& current_state_label = future.get()->current_state.label;
      if (current_state_label != state_name) {
        RCLCPP_DEBUG(node_->get_logger(), "Node: %s is in state: %s, expected: %s", node_name_.c_str(), current_state_label.c_str(), state_name.c_str());
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(node_->get_logger(), "Node: %s is in state: %s", node_name_.c_str(), state_name.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    private:
    // Node handle and name for logging purposes 
    rclcpp::Node::SharedPtr node_;
    std::string node_name_;

    // Client
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_;
  };

  class ChangeStateNode : public BT::SyncActionNode {
  public:
    ChangeStateNode(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
          : BT::SyncActionNode(name, config), node_(node)  {
      // Get key/value pair from input_ports
      auto it = config.input_ports.find("node_name");
      if (it == config.input_ports.end()) {
        throw std::runtime_error("ChangeStateNode: required input port 'node_name' not found");
      }
     
      // Extract the node_name from value
      node_name_ = it->second;
      
      client_ = node_->create_client<lifecycle_msgs::srv::ChangeState>("/" + node_name_ + "/change_state");
      
      RCLCPP_INFO(node_->get_logger(), "Client created for: %s", node_name_.c_str());
    }
    
    static BT::PortsList providedPorts(){
      return {
        BT::InputPort<std::string>("node_name"),
        BT::InputPort<std::string>("transition")
      };
    }

  BT::NodeStatus tick() override {
    // Get the transition name from input port
    auto transition_res = getInput<std::string>("transition");
    if (!transition_res) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input port [transition]: %s", transition_res.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Map transition name to transition ID 
    const auto& transition_name = transition_res.value();

    // Check if the transition name is valid
    auto transition_it = kTransitions.find(transition_name);
    if (transition_it == kTransitions.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Unknown lifecycle transition: %s", transition_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Create the service request
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_it->second;

    // Call the service and wait for the result
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "Service not available: /%s/change_state", node_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Send the request and wait for the response
    auto future = client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));

    // Check if the service call was successful
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed for node: %s", node_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Check the response
    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(node_->get_logger(), "Transition '%s' failed for node: %s", transition_name.c_str(), node_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Transition '%s' succeeded for node: %s", transition_name.c_str(), node_name_.c_str());

    return BT::NodeStatus::SUCCESS;
  }
      
  private:
    // Node handle and name for logging purposes 
    rclcpp::Node::SharedPtr node_;
    std::string node_name_;

    // Client
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;

    inline static const std::unordered_map<std::string, uint8_t> kTransitions {
      {"configure",   1},
      {"activate",    3},
      {"deactivate",  4},
      {"cleanup",     6}
    };
  };

// ==============================================================================
// ROS 2 Node
// ==============================================================================

  class BaseNode : public rclcpp::Node {
    public:
    BaseNode() : Node("base_node") {};

    void build() {
      // Register custom nodes with the factory
      BT::BehaviorTreeFactory factory;
      factory.registerBuilder<IsInState>(
        "IsInState",
        [this ](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<IsInState>(name, config, this->shared_from_this());
      });
      factory.registerBuilder<ChangeStateNode>(
        "ChangeStateNode",
        [this ](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<ChangeStateNode>(name, config, this->shared_from_this());
      });

      // Load the behavior tree from an XML file and calls constructor of the nodes
      std::string tree_path = ament_index_cpp::get_package_share_directory("mserve_bringup_bt") + "/trees/bringup.xml";
      auto tree = factory.createTreeFromFile(tree_path);

      // Execute the tree
      RCLCPP_INFO(this->get_logger(), "Starting behavior tree execution...");
      tree.tickWhileRunning();
    }
  };

}  // namespace lifecyclemanager_bt

// ==============================================================================
// Main
// ==============================================================================

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
 
  auto node = std::make_shared<lifecyclemanager_bt::BaseNode>();
  try
  {
    node->build();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  rclcpp::shutdown();
  return 0;
}
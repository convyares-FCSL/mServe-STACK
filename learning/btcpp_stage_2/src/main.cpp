#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
//#include <behaviortree_cpp/publisher_zmq.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include "mserve_utils/lifecycle.hpp"

// ==============================================================================
// Main entry point for bringup_bt.
// ==============================================================================

namespace lifecyclemanager_bt {

// ==============================================================================
// BT Action Node
// ==============================================================================
 
class IsInState : public BT::RosServiceNode<lifecycle_msgs::srv::GetState> {
  public:
    IsInState(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<lifecycle_msgs::srv::GetState>(name, config, params) {
      // Get key/value pair from input_ports
      auto it = config.input_ports.find("node_name");
      if (it == config.input_ports.end()) {
        throw std::runtime_error("IsInState: required input port 'node_name' not found");
      }
    
      // Extract the node_name from value
      node_name_ = it->second;
      
      // Set the service name based on the node name
      setServiceName("/" + node_name_ + "/get_state");
    }
    
    // Define the input ports for the node
    static BT::PortsList providedPorts() {
      return providedBasicPorts({
        BT::InputPort<std::string>("node_name"),
        BT::InputPort<std::string>("state")
      });
    }

    // Construct the service request based on the input ports and send it.
    bool setRequest(Request::SharedPtr& request) override {
      // Empty request
      (void)request;

      return true;
    }

    // Process the service response and return the appropriate BT status.
    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
      auto state_res = getInput<std::string>("state");
      if (!state_res) {
        RCLCPP_ERROR(logger(), "Missing input port [state]: %s", state_res.error().c_str());
        return BT::NodeStatus::FAILURE;
      }

      // Map state name to state ID 
      const auto& state_name = state_res.value();

      // Check if the current state matches the expected state
      if (response->current_state.label != state_name) {
        RCLCPP_INFO(logger(), "Node %s is NOT in state: %s (current state: %s)", node_name_.c_str(), state_name.c_str(), response->current_state.label.c_str());
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(logger(), "Node %s is in state: %s", node_name_.c_str(), state_name.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    private:
    std::string node_name_;
  };

class ChangeStateNode : public BT::RosServiceNode<lifecycle_msgs::srv::ChangeState> {
  public:
    ChangeStateNode(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<lifecycle_msgs::srv::ChangeState>(name, config, params) {
      // Get key/value pair from input_ports
      auto it = config.input_ports.find("node_name");
      if (it == config.input_ports.end()) {
        throw std::runtime_error("ChangeStateNode: required input port 'node_name' not found");
      }

      // Extract the node_name from value
      node_name_ = it->second;
      
      // Set the service name based on the node name
      setServiceName("/" + node_name_ + "/change_state");
     }

    // Define the input ports for the node
   static BT::PortsList providedPorts() {
    return providedBasicPorts({
      BT::InputPort<std::string>("node_name"),
      BT::InputPort<std::string>("transition")
    });
  }

    // Construct the service request based on the input ports and send it.
    bool setRequest(Request::SharedPtr& request) override {
      // Get the transition name from input port
      auto transition_res = getInput<std::string>("transition");
      if (!transition_res) {
        RCLCPP_ERROR(logger(), "Missing input port [transition]: %s", transition_res.error().c_str());
        return false;
      }

      // Map transition name to transition ID 
      const auto& transition_name = transition_res.value();

      // Check if the transition name is valid
      auto transition_it = mserve_utils::lifecycle::transitionIdFromName(transition_name);
      if (!transition_it) {
        RCLCPP_ERROR(logger(), "Unknown lifecycle transition: %s", transition_name.c_str());
        return false;
      }

      request->transition.id = transition_it.value();
      return true;
    }

    // Process the service response and return the appropriate BT status.
    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
      if (!response->success) {
        RCLCPP_ERROR(logger(), "Transition failed for node: %s", node_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(logger(), "Transition succeeded for node: %s", node_name_.c_str());
      return BT::NodeStatus::SUCCESS;
    }

  private:
    std::string node_name_;
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
        [this](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<IsInState>(name, config, BT::RosNodeParams(this->shared_from_this()));
      });
      factory.registerBuilder<ChangeStateNode>(
        "ChangeStateNode",
        [this](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<ChangeStateNode>(name, config, BT::RosNodeParams(this->shared_from_this()));
      });

      // Load the behavior tree from an XML file and calls constructor of the nodes
      std::string tree_path = ament_index_cpp::get_package_share_directory("mserve_bringup_bt") + "/trees/bringup.xml";
      auto tree = factory.createTreeFromFile(tree_path);

      /*
      // Create a ZMQ publisher to visualize the tree in Groot2
      BT::PublisherZMQ publisher(tree);
      */

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
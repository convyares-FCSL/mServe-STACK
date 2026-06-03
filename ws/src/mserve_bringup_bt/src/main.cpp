#include <cstdlib>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <lifecycle_msgs/srv/change_state.hpp>

// ==============================================================================
// Main entry point for bringup_bt.
// ==============================================================================

namespace lifecyclemanager_bt {

// ==============================================================================
// BT Action Node
// ==============================================================================

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
    
    static BT::PortsList providedPorts()
    {
    return {
      BT::InputPort<std::string>("node_name"),
      BT::InputPort<std::uint8_t>("transition")
    };
    }
      
    BT::NodeStatus tick() override {
      auto transition_res = getInput<std::uint8_t>("transition");
      if (!transition_res) {
        RCLCPP_ERROR(node_->get_logger(), "Missing input port [transition]: %s", transition_res.error().c_str());
        return BT::NodeStatus::FAILURE;
      }

      auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      request->transition.id = transition_res.value();

      if (!client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available: %s", node_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      auto future = client_->async_send_request(request);
      auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));

      if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed for node: %s", node_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      if (!future.get()->success) {
        RCLCPP_ERROR(node_->get_logger(), "Transition failed for node: %s", node_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }
 
      RCLCPP_INFO(node_->get_logger(), "Transition succeeded for node: %s", node_name_.c_str());
      return BT::NodeStatus::SUCCESS;
    } 

  private:
    // Node handle and name for logging purposes 
    rclcpp::Node::SharedPtr node_;
    std::string node_name_;

    // Client
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
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
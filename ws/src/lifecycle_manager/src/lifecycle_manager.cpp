#include "mserve_lifecycle_manager/lifecycle_manager.hpp"
#include "mserve_utils/lifecycle.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/xml_parsing.h>
#include <fstream>

namespace lifecyclemanager {

std::atomic<bool> LifecycleManager::shutdown_requested_{false};

namespace {
BT::RosNodeParams lifecycleRosParams(const std::shared_ptr<rclcpp::Node>& node) {
    BT::RosNodeParams params(node);
    params.wait_for_server_timeout = std::chrono::seconds(2);
    // 2s -> 10s (2026-07-19): slam_toolbox blocks its own executor
    // synchronously deserializing a saved pose-graph at configure time
    // (--slam-local) — house_v2's ~7MB posegraph took long enough to miss a
    // 2s get_state response window. That timeout becomes a FAILURE from
    // IsInState, which the bringup trees' Fallback[Inverter[IsInState],...]
    // pattern misreads as "already past this state, skip the transition" —
    // silently skipping configure *and* activate while the tree's own
    // computed status comes back genuine SUCCESS (see slam_bringup.xml).
    // Confirmed on real hardware: slam_toolbox was left stuck unconfigured,
    // /map never published, with the log claiming "All nodes successfully
    // activated" the whole time. A longer timeout doesn't fix the
    // Fallback/Inverter pattern's fragility under a real timeout (see the
    // bringup_complete_ fix below for that), but it removes the specific
    // trigger for any map of reasonable size — drivechain/base/camera/lidar
    // configure fast enough that this has no effect on them either way.
    params.server_timeout = std::chrono::seconds(10);
    return params;
}
}

// ==============================================================================
// BT checker nodes for lifecycle state management
// ==============================================================================
 
IsInState::IsInState(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<lifecycle_msgs::srv::GetState>(name, config, params) {
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

BT::PortsList IsInState::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("node_name"),
        BT::InputPort<std::string>("state")
    });
}

bool IsInState::setRequest(Request::SharedPtr& request) {
// Empty request
    (void)request;

    return true;
}

BT::NodeStatus IsInState::onResponseReceived(const Response::SharedPtr& response) {
    // Get the expected state from input port
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

// ==============================================================================
// BT action nodes for lifecycle state management
// ==============================================================================
    
ChangeStateNode::ChangeStateNode(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<lifecycle_msgs::srv::ChangeState>(name, config, params) {
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

BT::PortsList ChangeStateNode::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("node_name"),
        BT::InputPort<std::string>("transition")
    });
}

bool ChangeStateNode::setRequest(Request::SharedPtr& request) {
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

BT::NodeStatus ChangeStateNode::onResponseReceived(const Response::SharedPtr& response) {
    if (!response->success) {
        RCLCPP_ERROR(logger(), "Transition failed for node: %s", node_name_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "Transition succeeded for node: %s", node_name_.c_str());
    return BT::NodeStatus::SUCCESS;
    }

    
// ==============================================================================
// ROS 2 Lifecycle Manager class
// ==============================================================================
 
void LifecycleManager::build() {
    // Register custom nodes with the factory
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<IsInState>(
        "IsInState",
        [this](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsInState>(name, config, lifecycleRosParams(this->shared_from_this()));
    });
    factory.registerBuilder<ChangeStateNode>(
        "ChangeStateNode",
        [this](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ChangeStateNode>(name, config, lifecycleRosParams(this->shared_from_this()));
    });

    // Export the tree model to an XML file for visualization in Groot2
    std::string models_path = ament_index_cpp::get_package_share_directory("lifecycle_manager") + "/trees/node_models.xml";
    std::ofstream(models_path) << BT::writeTreeNodesModelXML(factory);

    // Which tree files to load — defaults match the always-on drivechain/
    // base/camera/lidar bringup. Overridden via params by callers managing a
    // different node set (e.g. mserve_slam.launch.py points a second,
    // separately-named instance of this same executable at slam_bringup.xml/
    // slam_shutdown.xml so opt-in nodes get the same retry/graceful-shutdown
    // behavior without adding dead-service-lookup delay to every normal
    // startup that isn't using them).
    std::string bringup_tree_file = declare_parameter<std::string>("bringup_tree_file", "bringup.xml");
    std::string shutdown_tree_file = declare_parameter<std::string>("shutdown_tree_file", "shutdown.xml");
    std::string trees_dir = ament_index_cpp::get_package_share_directory("lifecycle_manager") + "/trees/";

    // Load the behavior tree from an XML file
    std::string tree_path = trees_dir + bringup_tree_file;
    tree_ = factory.createTreeFromFile(tree_path);

    // Load the shutdown tree from an XML file
    std::string shutdown_path = trees_dir + shutdown_tree_file;
    shutdown_tree_ = factory.createTreeFromFile(shutdown_path);

    // Create a ZMQ publisher to visualize the tree in Groot2, if its default port is free.
    try {
        groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_);
    } catch (const std::exception& error) {
        RCLCPP_WARN(this->get_logger(), "Groot2 publisher disabled: %s", error.what());
        groot2_publisher_.reset();
    }

    // Execute the tree
    RCLCPP_INFO(this->get_logger(), "Starting behavior tree execution...");

    // A signal handler just flips shutdown_requested_ (see main.cpp) — the
    // actual shutdown tree runs here, from this timer, while the ROS context
    // is still valid. rclcpp::on_shutdown() fires too late for that: by then
    // the context is already torn down and RosServiceNode calls silently fail,
    // leaving mserve_drivechain/mserve_base stuck active. Once the shutdown
    // tree finishes, this callback calls rclcpp::shutdown() itself.
    tick_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (shutdown_requested_.load()) {
            if (!shutting_down_) {
                shutting_down_ = true;
                RCLCPP_INFO(get_logger(), "Shutdown signal received, running shutdown tree...");
            }
            auto status = shutdown_tree_.tickOnce();
            if (status != BT::NodeStatus::RUNNING) {
                RCLCPP_INFO(get_logger(), "Shutdown tree complete.");
                tick_timer_->cancel();
                rclcpp::shutdown();
            }
            return;
        }

        if (!bringup_complete_) {
            auto status = tree_.tickOnce();
            // Was `if (status != RUNNING)`, treating FAILURE the same as
            // SUCCESS — confirmed on real hardware (2026-07-19): a genuine
            // tree FAILURE (see lifecycleRosParams' comment on the
            // server_timeout bump for the actual trigger) still logged
            // "All nodes successfully activated" and set bringup_complete_,
            // permanently halting further ticks with a lifecycle node stuck
            // unconfigured. On FAILURE, deliberately *don't* set
            // bringup_complete_: BT.CPP re-descends from the root on the
            // next tickOnce() for any node not currently RUNNING (including
            // RetryUntilSuccessful's own attempt counter), so leaving this
            // false lets the 100ms timer above retry the whole bringup tree
            // again next tick, self-healing once the transient condition
            // (e.g. slam_toolbox finishing a slow configure-time load)
            // clears — no separate backoff/retry-limit logic needed here.
            if (status == BT::NodeStatus::SUCCESS) {
                bringup_complete_ = true;
                RCLCPP_INFO(get_logger(), "All nodes successfully activated.");
            } else if (status == BT::NodeStatus::FAILURE) {
                RCLCPP_ERROR(get_logger(), "Bringup tree failed this tick — retrying...");
            }
        }
    });
}

} // namespace lifecyclemanager

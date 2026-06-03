#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <iostream>

class SayHello : public BT::SyncActionNode {
public:
    SayHello(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {}    

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
        std::cout << "Hello, World!" << std::endl;
        return BT::NodeStatus::SUCCESS;
  }
};

int main() {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SayHello>("SayHello");

  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <SayHello name="test"/>
      </BehaviorTree>
    </root>
  )");

  tree.tickWhileRunning();  

  return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>

class IsInRamp : public BT::ConditionNode
{
public:
    IsInRamp(const std::string &name, const BT::NodeConfiguration &config) : BT::ConditionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        auto pose = getCurrentPose();

        if (pointInsideRamp(pose.x, pose.y))
            return BT::NodeStatus::SUCCESS;
        else
            return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

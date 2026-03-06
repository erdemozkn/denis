#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <map>

namespace denis_v1_bt
{

    struct Point
    {
        double x, y;
    };

    class CheckZone : public BT::ConditionNode
    {
    public:
        CheckZone(const std::string &name, const BT::NodeConfiguration &config)
            : BT::ConditionNode(name, config)
        {

            config.blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            std::string yaml_path = "/home/erdem/ros2_ws/src/denis_v1_bringup/rooms/ev.yaml";
            loadZonesFromYaml(yaml_path);
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("zone_name")};
        }

        BT::NodeStatus tick() override
        {
            std::string zone_name;
            if (!getInput("zone_name", zone_name))
                return BT::NodeStatus::FAILURE;

            if (all_zones_.find(zone_name) == all_zones_.end())
            {
                RCLCPP_ERROR(node_->get_logger(), "%s bölgesi YAML içinde bulunamadı!", zone_name.c_str());
                return BT::NodeStatus::FAILURE;
            }

            auto pose = getCurrentPose();
            if (isPointInPolygon(pose.position.x, pose.position.y, all_zones_[zone_name]))
            {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::map<std::string, std::vector<Point>> all_zones_;

        void loadZonesFromYaml(const std::string &path)
        {
            try
            {
                YAML::Node config = YAML::LoadFile(path);
                for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
                {
                    std::string zone_name = it->first.as<std::string>();
                    std::vector<Point> points;

                    auto pts_node = it->second["points"];
                    for (size_t i = 0; i < pts_node.size(); ++i)
                    {
                        points.push_back({pts_node[i][0].as<double>(), pts_node[i][1].as<double>()});
                    }
                    all_zones_[zone_name] = points;
                    RCLCPP_INFO(node_->get_logger(), "Bölge yüklendi: %s (%ld nokta)", zone_name.c_str(), points.size());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "YAML yükleme hatası: %s", e.what());
            }
        }

        bool isPointInPolygon(double x, double y, const std::vector<Point> &poly)
        {
            bool inside = false;
            for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
            {
                if (((poly[i].y > y) != (poly[j].y > y)) &&
                    (x < (poly[j].x - poly[i].x) * (y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
                    inside = !inside;
            }
            return inside;
        }

        geometry_msgs::msg::Pose getCurrentPose()
        {
            geometry_msgs::msg::Pose p;
            try
            {
                auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                p.position.x = t.transform.translation.x;
                p.position.y = t.transform.translation.y;
            }
            catch (...)
            {
            }
            return p;
        }
    };
} // namespace denis_v1_bt

#include <behaviortree_cpp/bt_factory.h>

extern "C"
{

    void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
    {

        factory.registerNodeType<denis_v1_bt::CheckZone>("CheckZone");
    }
}
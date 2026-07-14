#include <rclcpp/rclcpp.hpp>
#include <snapstack_msgs2/msg/goal.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GoalVisualizer : public rclcpp::Node
{
public:
    GoalVisualizer()
    : Node("goal_visualizer")
    {
        declare_parameter<std::string>("target_namespace", "");

        get_parameter("target_namespace", target_ns_);

        if (target_ns_.empty()) {
            target_ns_ = get_namespace();
        }

        // Normalize namespace string
        if (!target_ns_.empty() && target_ns_[0] == '/') {
            target_ns_.erase(0, 1);
        }

        std::string goal_topic = "/" + target_ns_ + "/goal";
        std::string marker_topic = "/" + target_ns_ + "/goal_marker";

        goal_sub_ = create_subscription<snapstack_msgs2::msg::Goal>(
            goal_topic,
            10,
            std::bind(&GoalVisualizer::goalCallback, this, std::placeholders::_1));

        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            marker_topic,
            10);

        RCLCPP_INFO(get_logger(), "GoalVisualizer listening on: %s", goal_topic.c_str());
        RCLCPP_INFO(get_logger(), "GoalVisualizer publishing on: %s", marker_topic.c_str());
    }

private:
    void goalCallback(const snapstack_msgs2::msg::Goal::SharedPtr msg)
    {
        visualization_msgs::msg::Marker marker;
        // marker.header = msg->header;
        marker.header.frame_id = "map";
        marker.ns = "goal_arrow";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Arrow starts at the goal position
        marker.pose.position = msg->p;

        // Convert psi (yaw) into quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, msg->psi);
        marker.pose.orientation = tf2::toMsg(q);

        // Scale:
        // x = arrow length, y/z = shaft diameter
        marker.scale.x = 0.8;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        // Red arrow
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // Keep it visible briefly, but refresh continuously
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_pub_->publish(marker);
    }

    std::string target_ns_;
    rclcpp::Subscription<snapstack_msgs2::msg::Goal>::SharedPtr goal_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
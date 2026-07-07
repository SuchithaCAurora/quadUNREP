#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <Eigen/Geometry>

using std::placeholders::_1;

class FollowerTrajGenerator : public rclcpp::Node
{
public:
    FollowerTrajGenerator()
    : Node("follower_traj_generator")
    {
        // Parameters
        this->declare_parameter<std::string>("follower_mode", "heading_based"); // either velocity_based or heading_based
        this->declare_parameter<double>("follower_distance_T", 1.0);
        this->declare_parameter<double>("follower_distance_N", 1.0);
        this->declare_parameter<double>("follower_distance_B", 1.0);
        this->declare_parameter<double>("vel_eps", 1e-3); // prevent division by 0
        this->declare_parameter<std::string>("leader_topic", "/SQ01/mavros/local_position/odom"); 

        this->get_parameter("follower_mode", follower_mode_);
        this->get_parameter("follower_distance_T", follower_distance_T_);
        this->get_parameter("follower_distance_N", follower_distance_N_);
        this->get_parameter("follower_distance_B", follower_distance_B_);
        this->get_parameter("vel_eps", vel_eps_);
        this->get_parameter("leader_topic", leader_topic_);

        if (follower_mode_ != "velocity_based" && follower_mode_ != "heading_based") {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid follower_mode='%s'. Falling back to 'velocity_based'.",
                follower_mode_.c_str());
            follower_mode_ = "velocity_based";
        }

        // QoS
        rclcpp::QoS qos_profile(10);
        qos_profile
            .durability(rclcpp::DurabilityPolicy::Volatile)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);

        leader_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            leader_topic_,
            qos_profile,
            std::bind(&FollowerTrajGenerator::leaderCB, this, _1));

        follower_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "follower_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&FollowerTrajGenerator::pubCB, this));
    }

private:
    void leaderCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        leader_odom_ = *msg;
        has_leader_ = true;
    }

    void pubCB()
    {
        if (!has_leader_) {
            return;
        }

        const auto& pos = leader_odom_.pose.pose.position;
        const auto& qmsg = leader_odom_.pose.pose.orientation;

        geometry_msgs::msg::PoseStamped follower;
        follower.header.stamp = this->now();
        follower.header.frame_id = leader_odom_.header.frame_id.empty()
            ? "map"
            : leader_odom_.header.frame_id;

        Eigen::Vector3d offset_base_link(
            -follower_distance_T_,
            -follower_distance_N_,
            -follower_distance_B_
        );

        Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);

        Eigen::Vector3d offset_map = q * offset_base_link;
        
        if (follower_mode_ == "heading_based") {
            follower.pose.position.x = pos.x + offset_map.x();
            follower.pose.position.y = pos.y + offset_map.y();
            follower.pose.position.z = pos.z + offset_map.z();
        }

        // Keep same orientation as leader for now
        follower.pose.orientation = qmsg;

        follower_pub_->publish(follower);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[%s] leader (%.2f, %.2f, %.2f) -> follower (%.2f, %.2f, %.2f)",
            follower_mode_.c_str(),
            pos.x, pos.y, pos.z,
            follower.pose.position.x,
            follower.pose.position.y,
            follower.pose.position.z);
    }

    double quat2yaw(const geometry_msgs::msg::Quaternion& q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follower_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry leader_odom_;
    bool has_leader_{false};

    std::string follower_mode_{"heading_based"};
    std::string leader_topic_{"/SQ01/mavros/local_position/odom"};
    double follower_distance_T_{0.0}; // give positive values as logic in rest of code will negate it (behind leader)
    double follower_distance_N_{1.0};
    double follower_distance_B_{0.0};
    double vel_eps_{1e-3};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerTrajGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
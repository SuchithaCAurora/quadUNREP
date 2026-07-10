#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <snapstack_msgs2/msg/goal.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <string>

using std::placeholders::_1;

class FollowerGoalGenerator : public rclcpp::Node
{
public:
    FollowerGoalGenerator()
    : Node("follower_goal_generator")
    {
        // Parameters
        this->declare_parameter<std::string>("follower_mode", "heading_based");
        this->declare_parameter<double>("follower_distance_T", 0.0);
        this->declare_parameter<double>("follower_distance_N", 0.0);
        this->declare_parameter<double>("follower_distance_B", 0.0);
        this->declare_parameter<double>("publish_freq", 100.0);
        this->declare_parameter<std::string>("leader_topic", "/SQ01/mavros/local_position/odom");
        this->declare_parameter<double>("goal_altitude", 3.0);
        // Initial position of follower drone with respect to leader drone in map frame
        this->declare_parameter<std::vector<double>>("init_follower_offset", {0.0, 3.0, 0.0}); 

        this->get_parameter("follower_mode", follower_mode_);
        this->get_parameter("follower_distance_T", follower_distance_T_);
        this->get_parameter("follower_distance_N", follower_distance_N_);
        this->get_parameter("follower_distance_B", follower_distance_B_);
        this->get_parameter("leader_topic", leader_topic_);
        this->get_parameter("goal_altitude", goal_altitude_);
        this->get_parameter("init_follower_offset", init_follower_offset_);

        double publish_freq;
        this->get_parameter("publish_freq", publish_freq);
        if (publish_freq <= 0.0) {
            publish_freq = 100.0; // Hz
        }
        dt_ = 1.0 / publish_freq;

        if (follower_mode_ != "velocity_based" && follower_mode_ != "heading_based") {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid follower_mode='%s'. Falling back to 'heading_based'.",
                        follower_mode_.c_str());
            follower_mode_ = "heading_based";
        }

        rclcpp::QoS qos_profile(10);
        qos_profile
            .durability(rclcpp::DurabilityPolicy::Volatile)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);

        leader_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            leader_topic_,
            qos_profile,
            std::bind(&FollowerGoalGenerator::leaderCB, this, _1));

        goal_pub_ = this->create_publisher<snapstack_msgs2::msg::Goal>("goal", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&FollowerGoalGenerator::pubCB, this));

        RCLCPP_INFO(this->get_logger(), "FollowerGoalGenerator started.");
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

        Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);

        // Offset in leader body frame: behind leader in T, and optionally lateral/vertical offsets
        Eigen::Vector3d offset_base(
            -follower_distance_T_,
            -follower_distance_N_,
            -follower_distance_B_
        );

        Eigen::Vector3d init_pos_offset(
            init_follower_offset_[0],
            init_follower_offset_[1],
            init_follower_offset_[2]
        );

        Eigen::Vector3d offset_world = q * offset_base - init_pos_offset;

        snapstack_msgs2::msg::Goal goal;
        goal.header.stamp = this->now();

        // position target
        goal.p.x = pos.x + offset_world.x();
        goal.p.y = pos.y + offset_world.y();
        goal.p.z = goal_altitude_; //pos.z + offset_world.z();

        // velocity / accel / jerk can be zero for now
        goal.v.x = 0.0;
        goal.v.y = 0.0;
        goal.v.z = 0.0;

        goal.a.x = 0.0;
        goal.a.y = 0.0;
        goal.a.z = 0.0;

        goal.j.x = 0.0;
        goal.j.y = 0.0;
        goal.j.z = 0.0;

        // same yaw as leader
        goal.psi = quat2yaw(qmsg);
        goal.dpsi = 0.0;

        // This is important for your existing FSM / offboard logic
        goal.power = true;
        goal.mode_xy = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
        goal.mode_z = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;

        goal_pub_->publish(goal);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Leader (%.2f, %.2f, %.2f) -> Goal (%.2f, %.2f, %.2f)",
            pos.x, pos.y, pos.z,
            goal.p.x, goal.p.y, goal.p.z);
    }

    double quat2yaw(const geometry_msgs::msg::Quaternion& q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_state_sub_;
    rclcpp::Publisher<snapstack_msgs2::msg::Goal>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry leader_odom_;
    bool has_leader_{false};

    std::string follower_mode_{"heading_based"};
    std::string leader_topic_{"/SQ01/mavros/local_position/odom"};
    double follower_distance_T_;
    double follower_distance_N_;
    double follower_distance_B_;
    double goal_altitude_;
    double dt_{0.01};
    std::vector<double> init_follower_offset_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerGoalGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
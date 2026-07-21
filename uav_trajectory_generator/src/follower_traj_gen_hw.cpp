#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <snapstack_msgs2/msg/goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <string>

using std::placeholders::_1;

class FollowerGoalGeneratorHW : public rclcpp::Node
{
public:
    FollowerGoalGeneratorHW()
    : Node("follower_goal_generator_hw")
    {
        this->declare_parameter<std::string>("follower_mode", "heading_based");
        this->declare_parameter<double>("follower_distance_T", 0.0);
        this->declare_parameter<double>("follower_distance_N", 0.0);
        this->declare_parameter<double>("follower_distance_B", 0.0);
        this->declare_parameter<double>("publish_freq", 100.0);
        this->declare_parameter<std::string>("leader_topic_pos", "/PX03/world");
        this->declare_parameter<std::string>("leader_topic_vel", "/PX03/mocap/twist");
        this->declare_parameter<double>("goal_altitude", 3.0);
        this->declare_parameter<double>("max_speed", 1.2);
        this->declare_parameter<double>("max_accel", 1.0);
        this->declare_parameter<double>("filter_wn", 1.8);
        this->declare_parameter<double>("deadzone_vctrl", 0.03);
        this->declare_parameter<std::string>("follower_topic_pos", "world");
        this->declare_parameter<double>("leader_vel_timeout", 0.3);
        this->declare_parameter<std::string>("mavros_state_topic", "mavros/state");
        this->declare_parameter<double>("divergence_thresh", 1.0);

        this->declare_parameter<std::vector<double>>("init_follower_offset", {0.0, 3.0, 0.0});

        this->get_parameter("follower_mode", follower_mode_);
        this->get_parameter("follower_distance_T", follower_distance_T_);
        this->get_parameter("follower_distance_N", follower_distance_N_);
        this->get_parameter("follower_distance_B", follower_distance_B_);
        this->get_parameter("leader_topic_pos", leader_topic_pos_);
        this->get_parameter("leader_topic_vel", leader_topic_vel_);
        this->get_parameter("goal_altitude", goal_altitude_);
        this->get_parameter("init_follower_offset", init_follower_offset_);
        this->get_parameter("max_speed", v_max_);
        this->get_parameter("max_accel", a_max_);
        this->get_parameter("filter_wn", wn_);
        this->get_parameter("deadzone_vctrl", deadband_);
        this->get_parameter("follower_topic_pos", follower_topic_pos_);
        this->get_parameter("leader_vel_timeout", leader_vel_timeout_);
        this->get_parameter("mavros_state_topic", mavros_state_topic_);
        this->get_parameter("divergence_thresh", divergence_thresh_);

        double publish_freq;
        this->get_parameter("publish_freq", publish_freq);
        if (publish_freq <= 0.0) publish_freq = 100.0;
        dt_ = 1.0 / publish_freq;

        // Feasibility check: capture distance must exceed stopping distance
        const double a_required = wn_ * v_max_ / 4.0;
        if (a_max_ < a_required) {
            RCLCPP_WARN(this->get_logger(),
                "max_accel=%.2f is below wn*v_max/4=%.2f. Expect overshoot on capture. "
                "Lower filter_wn to <= %.2f or lower max_speed.",
                a_max_, a_required, 4.0 * a_max_ / v_max_);
        }

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

        leader_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            leader_topic_pos_, qos_profile,
            std::bind(&FollowerGoalGeneratorHW::leader_posCB, this, _1));

        leader_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            leader_topic_vel_, qos_profile,
            std::bind(&FollowerGoalGeneratorHW::leader_velCB, this, _1));

        self_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            follower_topic_pos_, qos_profile,
            std::bind(&FollowerGoalGeneratorHW::follower_posCB, this, _1));

        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            mavros_state_topic_, qos_profile,
            std::bind(&FollowerGoalGeneratorHW::mavrosStateCB, this, _1));

        goal_pub_ = this->create_publisher<snapstack_msgs2::msg::Goal>("goal", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&FollowerGoalGeneratorHW::pubCB, this));

        RCLCPP_INFO(this->get_logger(),
            "FollowerGoalGeneratorHW started. v_max=%.2f a_max=%.2f wn=%.2f",
            v_max_, a_max_, wn_);
    }

private:

    void leader_posCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        leader_pos_ = *msg;
        has_leader_ = true;
    }

    void leader_velCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        leader_vel_ = *msg;
        has_leader_vel_ = true;
        last_leader_vel_time_ = this->now();
    }

    void follower_posCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        follower_pos_ = *msg;
        has_follower_ = true;
    }

    void mavrosStateCB(const mavros_msgs::msg::State::SharedPtr msg)
    {
        const bool offboard_now = msg->armed && (msg->mode == "OFFBOARD");
        if (offboard_now && !offboard_prev_) {
            RCLCPP_INFO(this->get_logger(), "OFFBOARD engaged: filter will (re)initialize at current position.");
            sp_init_ = false;
        }
        offboard_prev_ = offboard_now;
        offboard_now_  = offboard_now;
        has_mavros_state_ = true;
    }

    static Eigen::Vector3d clipNorm(const Eigen::Vector3d& v, double lim)
    {
        const double n = v.norm();
        return (n > lim && n > 1e-9) ? (v * (lim / n)) : v;
    }

    void pubCB()
    {
        if (!has_leader_ || !has_follower_ || !has_leader_vel_) {
            return;
        }

        const auto& L_pos  = leader_pos_.pose.position;
        const auto& L_qmsg = leader_pos_.pose.orientation;
        const auto& L_vel  = leader_vel_.twist.linear;
        const auto& F_pos  = follower_pos_.pose.position;

        Eigen::Quaterniond L_q = quatFromMsg(L_qmsg);

        Eigen::Vector3d leader_pos_world(L_pos.x, L_pos.y, L_pos.z);
        Eigen::Vector3d follower_pos_world(F_pos.x, F_pos.y, F_pos.z);
        Eigen::Vector3d leader_vel_world(L_vel.x, L_vel.y, L_vel.z);

        // Stale leader velocity -> treat as zero rather than extrapolating
        const double vel_age = (this->now() - last_leader_vel_time_).seconds();
        if (vel_age > leader_vel_timeout_) {
            leader_vel_world.setZero();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Leader velocity stale (%.2f s).", vel_age);
        }

        Eigen::Vector3d desired_offset_tnb(
            follower_distance_T_,
            follower_distance_N_,
            follower_distance_B_);

        // Target: desired follower position in world frame
        Eigen::Vector3d target = leader_pos_world + L_q * desired_offset_tnb;

        // Not yet in OFFBOARD: hold in place and keep the filter un-initialized so it
        // doesn't integrate toward the target while the vehicle is on the ground / in
        // manual flight. PX4 still needs a continuous setpoint stream to accept the
        // mode switch, so we keep publishing.
        if (!has_mavros_state_ || !offboard_now_) {
            sp_init_ = false;

            snapstack_msgs2::msg::Goal hold;
            hold.header.stamp = this->now();
            hold.p.x = F_pos.x; hold.p.y = F_pos.y; hold.p.z = F_pos.z;
            hold.v.x = 0.0; hold.v.y = 0.0; hold.v.z = 0.0;
            hold.a.x = 0.0; hold.a.y = 0.0; hold.a.z = 0.0;
            hold.j.x = 0.0; hold.j.y = 0.0; hold.j.z = 0.0;
            hold.psi  = quat2yaw(L_qmsg);
            hold.dpsi = 0.0;
            hold.power   = true;
            hold.mode_xy = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
            hold.mode_z  = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
            goal_pub_->publish(hold);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for OFFBOARD: holding at current position.");
            return;
        }

        // ---- Second-order setpoint filter ----
        if (!sp_init_) {
            sp_ = follower_pos_world;
            sp_vel_.setZero();
            sp_init_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Filter initialized at (%.2f, %.2f, %.2f)",
                sp_.x(), sp_.y(), sp_.z());
        } else if ((sp_ - follower_pos_world).norm() > divergence_thresh_) {
            // Filter state has drifted too far from the actual vehicle position
            // (missed re-init edge, long dropout, etc.) -- snap back rather than
            // fly the accumulated error.
            RCLCPP_WARN(this->get_logger(),
                "Filter diverged from actual position by %.2f m (> %.2f m). Re-initializing.",
                (sp_ - follower_pos_world).norm(), divergence_thresh_);
            sp_ = follower_pos_world;
            sp_vel_.setZero();
        }

        Eigen::Vector3d err = target - sp_;
        if (err.norm() < deadband_) {
            err.setZero();
        }

        Eigen::Vector3d a_cmd = wn_ * wn_ * err - 2.0 * wn_ * sp_vel_;
        a_cmd = clipNorm(a_cmd, a_max_);

        sp_vel_ += a_cmd * dt_;
        sp_vel_  = clipNorm(sp_vel_, v_max_);
        sp_     += sp_vel_ * dt_;
        // --------------------------------------

        snapstack_msgs2::msg::Goal goal;
        goal.header.stamp = this->now();

        goal.p.x = sp_.x();
        goal.p.y = sp_.y();
        goal.p.z = sp_.z();

        goal.v.x = sp_vel_.x();
        goal.v.y = sp_vel_.y();
        goal.v.z = sp_vel_.z();

        goal.a.x = a_cmd.x();
        goal.a.y = a_cmd.y();
        goal.a.z = a_cmd.z();

        goal.j.x = 0.0;
        goal.j.y = 0.0;
        goal.j.z = 0.0;

        goal.psi  = quat2yaw(L_qmsg);
        goal.dpsi = 0.0;

        goal.power   = true;
        goal.mode_xy = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
        goal.mode_z  = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;

        goal_pub_->publish(goal);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "target (%.2f %.2f %.2f) | sp (%.2f %.2f %.2f) | sp_vel (%.2f %.2f %.2f) | "
            "actual (%.2f %.2f %.2f) | lag %.2f",
            target.x(), target.y(), target.z(),
            sp_.x(), sp_.y(), sp_.z(),
            sp_vel_.x(), sp_vel_.y(), sp_vel_.z(),
            follower_pos_world.x(), follower_pos_world.y(), follower_pos_world.z(),
            (target - sp_).norm());
    }

    double quat2yaw(const geometry_msgs::msg::Quaternion& q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    static Eigen::Quaterniond quatFromMsg(const geometry_msgs::msg::Quaternion& qmsg)
    {
        Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
        q.normalize();
        return q;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr leader_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr self_pos_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
    rclcpp::Publisher<snapstack_msgs2::msg::Goal>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped leader_pos_;
    geometry_msgs::msg::PoseStamped follower_pos_;
    geometry_msgs::msg::TwistStamped leader_vel_;
    bool has_leader_{false};
    bool has_follower_{false};
    bool has_leader_vel_{false};
    rclcpp::Time last_leader_vel_time_{0, 0, RCL_ROS_TIME};

    // Filter state
    Eigen::Vector3d sp_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d sp_vel_{Eigen::Vector3d::Zero()};
    bool sp_init_{false};

    // Offboard gating (mavros/state: armed + mode == "OFFBOARD")
    bool has_mavros_state_{false};
    bool offboard_now_{false};
    bool offboard_prev_{false};
    std::string mavros_state_topic_;
    double divergence_thresh_;

    double v_max_;
    double a_max_;
    double wn_;
    double deadband_;
    double leader_vel_timeout_;

    std::string follower_mode_{"heading_based"};
    std::string leader_topic_pos_;
    std::string leader_topic_vel_;
    std::string follower_topic_pos_;
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
    auto node = std::make_shared<FollowerGoalGeneratorHW>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
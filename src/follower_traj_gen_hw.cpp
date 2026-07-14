#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <snapstack_msgs2/msg/goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <string>

#include <iostream>

// Uncomment if using transforms
// #include <rclcpp/rclcpp.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/transform.h>


using std::placeholders::_1;

class FollowerGoalGeneratorHW : public rclcpp::Node
{
public:
    FollowerGoalGeneratorHW()
    : Node("follower_goal_generator_hw")
    {
        // Parameters
        this->declare_parameter<std::string>("follower_mode", "heading_based");
        this->declare_parameter<double>("follower_distance_T", 0.0);
        this->declare_parameter<double>("follower_distance_N", 0.0);
        this->declare_parameter<double>("follower_distance_B", 0.0);
        this->declare_parameter<double>("publish_freq", 100.0);
        this->declare_parameter<std::string>("leader_topic_pos", "/PX03/world");
        this->declare_parameter<std::string>("leader_topic_vel", "/PX03/mocap/twist");
        this->declare_parameter<double>("goal_altitude", 3.0);
        this->declare_parameter<double>("tuning_param", 0.01);
        this->declare_parameter<double>("max_speed", 2.0);
        this->declare_parameter<double>("deadzone_vctrl", 0.05);
        this->declare_parameter<std::string>("follower_topic_pos", "world");

        // Initial position of follower drone with respect to map frame
        this->declare_parameter<std::vector<double>>("init_follower_offset", {0.0, 3.0, 0.0}); 

        this->get_parameter("follower_mode", follower_mode_);
        this->get_parameter("follower_distance_T", follower_distance_T_);
        this->get_parameter("follower_distance_N", follower_distance_N_);
        this->get_parameter("follower_distance_B", follower_distance_B_);
        this->get_parameter("leader_topic_pos", leader_topic_pos_);
        this->get_parameter("leader_topic_vel", leader_topic_vel_);
        this->get_parameter("goal_altitude", goal_altitude_);
        this->get_parameter("init_follower_offset", init_follower_offset_);
        this->get_parameter("tuning_param", p_tuning_);
        this->get_parameter("max_speed", u_follower_max_);
        this->get_parameter("deadzone_vctrl", deadband_);
        this->get_parameter("follower_topic_pos", follower_topic_pos_);

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

        leader_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            leader_topic_pos_,
            qos_profile,
            std::bind(&FollowerGoalGeneratorHW::leader_posCB, this, _1));

        leader_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            leader_topic_vel_,
            qos_profile,
            std::bind(&FollowerGoalGeneratorHW::leader_velCB, this, _1));

        self_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            follower_topic_pos_,
            qos_profile,
            std::bind(&FollowerGoalGeneratorHW::follower_posCB, this, _1));

        goal_pub_ = this->create_publisher<snapstack_msgs2::msg::Goal>("goal", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&FollowerGoalGeneratorHW::pubCB, this));

        RCLCPP_INFO(this->get_logger(), "FollowerGoalGeneratorHW started.");
    }

private:

    // apply a deadband for the velocity commanding
    double applyDeadband(double e, double db)
    {
        if (std::fabs(e) < db) {
            return 0.0;
        }
        return e;
    }

    void leader_posCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        leader_pos_ = *msg;
        has_leader_ = true;
    }

    void leader_velCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        leader_vel_ = *msg;
    }

    void follower_posCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        follower_pos_ = *msg;
        has_follower_ = true;
    }

    // Publish the goal for the follower
    void pubCB()
    {
        if (!has_leader_ || !has_follower_) {
            return;
        }

        const auto& L_pos = leader_pos_.pose.position; // now is world frame
        const auto& L_qmsg = leader_pos_.pose.orientation; // now in world frame
        const auto& L_vel = leader_vel_.twist.linear; // now in world frame

        const auto& F_pos = follower_pos_.pose.position; // now in world frame

        // if both L_pos and F_pos are in same global frame:
            // Eigen::Vector3d desired_offset_world = L_q * desired_offset_tnb
            // Eigen::Vector3d desired_p_follower = leader_pos_world + desired_offset_world
        // or if using tf tools:
            // set the transforms for initial position using mocap
            // auto node = std::make_shared<rclcpp::Node>("transform_listener");

            // auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
            // auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node);

            // geometry_msgs::msg::TransformStamped transform;
            // try {
            //     transform = buffer->lookupTransform(
            //     "map",           // target frame
            //     "base_link",     // source frame
            //     rclcpp::Time()   // time to wait for transform
            //     ); // gives rotation from source to target

            // then use this transform for multiplication with vectors

        Eigen::Quaterniond L_q = quatFromMsg(L_qmsg);

        // Leader/follower positions (note here map and world are the same - TODO: go back and use consistent wording)
        Eigen::Vector3d leader_pos_world(L_pos.x, L_pos.y, L_pos.z);
        Eigen::Vector3d follower_pos_world(F_pos.x, F_pos.y, F_pos.z);

        // Offset in leader body frame: behind leader in T, and optionally lateral/vertical offsets
        Eigen::Vector3d desired_offset_tnb(
            follower_distance_T_,
            follower_distance_N_,
            follower_distance_B_
        );

        // desired position of the follower in the world frame
        Eigen::Vector3d desired_p_follower = leader_pos_world + L_q * desired_offset_tnb;


        // Relative displacement from follower to leader in world
        Eigen::Vector3d rel_world = follower_pos_world - leader_pos_world;
        // Express relative displacement in leader body frame (TNB)
        Eigen::Vector3d rel_tnb = L_q.inverse() * rel_world;

        // Formation error in leader frame
        Eigen::Vector3d e_form = desired_offset_tnb - rel_tnb;

        // Leader velocity (note that twist stamped message is expressed in world frame)
        // Eigen::Vector3d leader_vel_tnb(L_vel.x, L_vel.y, L_vel.z);
        Eigen::Vector3d leader_vel_tnb = L_q.inverse() * leader_vel_world; // use this if the linear velocity message was in map frame

        double sT = applyDeadband(e_form.x(), deadband_);
        double sN = applyDeadband(e_form.y(), deadband_);
        double sB = applyDeadband(e_form.z(), deadband_);
        double uT = leader_vel_tnb.x() + u_follower_max_ * sT / std::sqrt(sT*sT + p_tuning_ * p_tuning_);
        double uN = leader_vel_tnb.y() + u_follower_max_ * sN / std::sqrt(sN*sN + p_tuning_ * p_tuning_);
        double uB = leader_vel_tnb.z() + u_follower_max_ * sB / std::sqrt(sB*sB + p_tuning_ * p_tuning_);

        Eigen::Vector3d v_TNB(uT,uN,uB);
        double norm = v_TNB.norm();
        if (norm > u_follower_max_) {
            v_TNB *= (u_follower_max_ / norm);
        }
        // Get velocity back into world frame
        Eigen::Vector3d v_world = L_q * v_TNB;

        snapstack_msgs2::msg::Goal goal;
        goal.header.stamp = this->now();

        // position target
        goal.p.x = desired_p_follower.x(); 
        goal.p.y = desired_p_follower.y(); 
        goal.p.z = desired_p_follower.z(); 

        // accel / jerk can be zero for now

        // goal.v.x = 0.0;//
        // goal.v.y = 0.0;//
        // goal.v.z = 0.0;//

        goal.v.x = v_world.x();
        goal.v.y = v_world.y();
        goal.v.z = v_world.z();

        goal.a.x = 0.0;
        goal.a.y = 0.0;
        goal.a.z = 0.0;

        goal.j.x = 0.0;
        goal.j.y = 0.0;
        goal.j.z = 0.0;

        // same yaw as leader
        goal.psi = quat2yaw(L_qmsg);
        goal.dpsi = 0.0;

        goal.power = true;
        goal.mode_xy = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
        goal.mode_z = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;

        goal_pub_->publish(goal);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Leader (%.2f, %.2f, %.2f) -> Goal (%.2f, %.2f, %.2f)",
            L_pos.x, L_pos.y, L_pos.z,
            goal.p.x, goal.p.y, goal.p.z);
    }

    double quat2yaw(const geometry_msgs::msg::Quaternion& q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    static Eigen::Quaterniond quatFromMsg(const geometry_msgs::msg::Quaternion & qmsg)
    {
        Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
        q.normalize();
        return q;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr leader_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr self_pos_sub_;
    rclcpp::Publisher<snapstack_msgs2::msg::Goal>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped leader_pos_;
    geometry_msgs::msg::PoseStamped follower_pos_;
    geometry_msgs::msg::TwistStamped leader_vel_;
    bool has_leader_{false};
    bool has_follower_{false};

    // for velocity commands
    double u_follower_max_; // max speed m/s which follower can approach leader
    double L_T_vel_;
    double L_N_vel_;
    double L_B_vel_;

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
    double p_tuning_;
    double deadband_;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerGoalGeneratorHW>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <snapstack_msgs2/msg/goal.hpp>

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
        this->declare_parameter<double>("tuning_param", 0.01);
        this->declare_parameter<double>("max_speed", 2.0);
        this->declare_parameter<double>("deadzone_vctrl", 0.05);
        this->declare_parameter<std::string>("follower_topic", "mavros/local_position/odom");

        // Initial position of follower drone with respect to map frame
        this->declare_parameter<std::vector<double>>("init_follower_offset", {0.0, 3.0, 0.0}); 

        this->get_parameter("follower_mode", follower_mode_);
        this->get_parameter("follower_distance_T", follower_distance_T_);
        this->get_parameter("follower_distance_N", follower_distance_N_);
        this->get_parameter("follower_distance_B", follower_distance_B_);
        this->get_parameter("leader_topic", leader_topic_);
        this->get_parameter("goal_altitude", goal_altitude_);
        this->get_parameter("init_follower_offset", init_follower_offset_);
        this->get_parameter("tuning_param", p_tuning_);
        this->get_parameter("max_speed", u_follower_max_);
        this->get_parameter("deadzone_vctrl", deadband_);
        this->get_parameter("follower_topic", follower_topic_);

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

        self_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            follower_topic_, //"mavros/local_position/odom",
            qos_profile,
            std::bind(&FollowerGoalGenerator::followerCB, this, _1));

        goal_pub_ = this->create_publisher<snapstack_msgs2::msg::Goal>("goal", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&FollowerGoalGenerator::pubCB, this));

        RCLCPP_INFO(this->get_logger(), "FollowerGoalGenerator started.");
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

    void leaderCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        leader_odom_ = *msg;
        has_leader_ = true;
    }

    void followerCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        follower_odom_ = *msg;
        has_follower_ = true;
    }

    // Publish the goal for the follower
    void pubCB()
    {
        if (!has_leader_ || !has_follower_) {
            return;
        }

        const auto& L_pos = leader_odom_.pose.pose.position;
        const auto& L_qmsg = leader_odom_.pose.pose.orientation;
        const auto& L_vel = leader_odom_.twist.twist.linear;

        const auto& F_pos = follower_odom_.pose.pose.position;

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
        Eigen::Vector3d follower_pos_local(F_pos.x, F_pos.y, F_pos.z);

        // Offset in leader body frame: behind leader in T, and optionally lateral/vertical offsets
        Eigen::Vector3d desired_offset_tnb(
            follower_distance_T_,
            follower_distance_N_,
            follower_distance_B_
        );
        // Apply startup correction so follower is placed into the shared world frame
        Eigen::Vector3d init_offset_world(
            init_follower_offset_[0],
            init_follower_offset_[1],
            init_follower_offset_[2]);

        // desired position of the follower in the world frame
        Eigen::Vector3d desired_p_follower = leader_pos_world + L_q * desired_offset_tnb - init_offset_world;


        Eigen::Vector3d follower_pos_world = follower_pos_local + init_offset_world;
        // Relative displacement from follower to leader in world
        Eigen::Vector3d rel_world = follower_pos_world - leader_pos_world;
        // Express relative displacement in leader body frame (TNB)
        Eigen::Vector3d rel_tnb = L_q.inverse() * rel_world;

        // Formation error in leader frame
        Eigen::Vector3d e_form = desired_offset_tnb - rel_tnb;

        // Leader velocity (note that odom twist message is expressed in childe_frame_id)
        Eigen::Vector3d leader_vel_tnb(L_vel.x, L_vel.y, L_vel.z);
        // Eigen::Vector3d leader_vel_tnb = L_q.inverse() * leader_vel_world; // use this if the linear velocity message was in map frame

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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr self_state_sub_;
    rclcpp::Publisher<snapstack_msgs2::msg::Goal>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry leader_odom_;
    nav_msgs::msg::Odometry follower_odom_;
    bool has_leader_{false};
    bool has_follower_{false};

    // for velocity commands
    double u_follower_max_; // max speed m/s which follower can approach leader
    double L_T_vel_;
    double L_N_vel_;
    double L_B_vel_;

    std::string follower_mode_{"heading_based"};
    std::string leader_topic_;
    std::string follower_topic_;
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
    auto node = std::make_shared<FollowerGoalGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
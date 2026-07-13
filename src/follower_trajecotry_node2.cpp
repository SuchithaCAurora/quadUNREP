#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <snapstack_msgs2/msg/goal.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <string>

#include <iostream>

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
            "mavros/local_position/odom",
            qos_profile,
            std::bind(&FollowerGoalGenerator::followerCB, this, _1));

        goal_pub_ = this->create_publisher<snapstack_msgs2::msg::Goal>("goal", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&FollowerGoalGenerator::pubCB, this));

        RCLCPP_INFO(this->get_logger(), "FollowerGoalGenerator started.");
    }

private:
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

    void pubCB()
    {
        if (!has_leader_ || !has_follower_) {
            return;
        }

        const auto& L_pos = leader_odom_.pose.pose.position;
        const auto& L_qmsg = leader_odom_.pose.pose.orientation;
        const auto& L_vel = leader_odom_.twist.twist.linear;

        const auto& F_pos = follower_odom_.pose.pose.position;
        const auto& F_qmsg = follower_odom_.pose.pose.orientation;

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
        // Eigen::Vector3d leader_vel_tnb = L_q.inverse() * leader_vel_world;

        double sT = applyDeadband(e_form.x(), 0.05);
        double sN = applyDeadband(e_form.y(), 0.05);
        double sB = applyDeadband(e_form.z(), 0.05);
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

        // Eigen::Vector3d offset_world = L_q * offset_base - init_pos_offset;

        snapstack_msgs2::msg::Goal goal;
        goal.header.stamp = this->now();

        // position target
        goal.p.x = desired_p_follower.x(); //L_pos.x + offset_world.x();
        goal.p.y = desired_p_follower.y(); //L_pos.y + offset_world.y();
        goal.p.z = desired_p_follower.z(); //L_pos.z + offset_world.z();

        // velocity / accel / jerk can be zero for now

        // compute desired velocities
        

        // Decompose follower pose into the TNB coordinate system aligned with body of leader
        // Requires listening to the leader's velocity and knowing own current position
            // extract from twist
        // get change of basis matrix'
        // geometry_msgs::msg::TransformStamped transform;
        // // TODO: how do I access the namespace of this node? Call it ns_
        // if (listener.canTransform(leader_ns_+"/base_link", ns_+"/base_link", ros::Time(0), ros::Duration(1.0))){
        //     transform = listener.lookupTransform(leader_ns_+"/base_link", ns_+"/base_link", ros::Time(0));
        // }
        // Eigen::Vector3d displacement_mapframe(L_pos.x - F_pos.x, L_pos.y - F_pos.y, L_pos.z - F_pos.z);
        // displacement_mapframe = displacement_mapframe + init_pos_offset;
        // Eigen::Vector3d displacement_Lframe = tf2::transformPoint(displacement_mapframe, transform);

        // double tuning_param = 1.0;

        // L_T_vel_ = L_vel.x + u_follower_max * displacement_Lframe[0]/sqrt(displacement_Lframe*displacement_Lframe + tuning_param*tuning_param);
        // L_N_vel_ = L_vel.y + u_follower_max * displacement_Lframe[1]/sqrt(displacement_Lframe*displacement_Lframe + tuning_param*tuning_param);
        // L_B_vel_ = L_vel.z + + u_follower_max * displacement_Lframe[2]/sqrt(displacement_Lframe*displacement_Lframe + tuning_param*tuning_param);
        // Eigen::Vector3d S(L_T_vel_, L_N_vel_, L_B_vel_);
        // // The above is still in the frame of the leader. Transform back to the follower's frame so that the commands can be given to the follower
        // if (listener.canTransform(ns_+"/base_link", leader_ns_+"/base_link", ros::Time(0), ros::Duration(1.0))){
        //     transform_to_F = listener.lookupTransform(ns_+"/base_link", leader_ns_+"/base_link", ros::Time(0));
        // }
        // Eigen::Vector3d follower_velocities = tf2::transformPoint(S, transform_to_F);

        // goal.v.x = 0.0;//
        // goal.v.y = 0.0;//
        // goal.v.z = 0.0;//

        goal.v.x = v_world.x();
        goal.v.y = v_world.y();
        goal.v.z = v_world.z();

        // END OF NEW LOGIC

        goal.a.x = 0.0;
        goal.a.y = 0.0;
        goal.a.z = 0.0;

        goal.j.x = 0.0;
        goal.j.y = 0.0;
        goal.j.z = 0.0;

        // same yaw as leader
        goal.psi = quat2yaw(L_qmsg);
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
    double follower_distance_T_;
    double follower_distance_N_;
    double follower_distance_B_;
    double goal_altitude_;
    double dt_{0.01};
    std::vector<double> init_follower_offset_;
    double p_tuning_;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerGoalGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
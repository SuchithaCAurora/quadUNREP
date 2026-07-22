#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <snapstack_msgs2/msg/goal.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <string>

#include <iostream>
#include <algorithm>

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

    snapstack_msgs2::msg::Goal simpleInterpolation(const Eigen::Vector3d& current_pos,double current_psi,
        const Eigen::Vector3d& current_vel, const snapstack_msgs2::msg::Goal& dest_pos, double dest_yaw, const Eigen::Vector3d& desired_vel, double vel_yaw,
        double dist_thresh, double yaw_thresh, double dt, bool& finished)
    {
        // this requires having the goal in the world frame and current position in the world frame
        // also the velocities in the world frame
        snapstack_msgs2::msg::Goal goal;
        // interpolate from current goal pos to the initial goal pos
        double Dx = dest_pos.p.x - current_pos.x();
        double Dy = dest_pos.p.y - current_pos.y();
        double dist = sqrt(Dx*Dx + Dy*Dy);
        double delta_yaw = dest_yaw - current_psi;
        delta_yaw = wrap(delta_yaw);

        bool dist_far = dist > dist_thresh;
        bool yaw_far  = fabs(delta_yaw) > yaw_thresh;
        finished = not dist_far and not yaw_far;  // both are close

        double accel_for_vel = 0.1;

        goal.p.z = dest_pos.p.z;  // this should be alt_ and the altitude where the drone took off too
        // are we too far from the dest?
        if(dist_far){
            double c = Dx/dist; // separation displacement (component of unit vector)
            double s = Dy/dist; // separation displacement (component of unit vector)

            goal.p.x = current_pos.x() + c*std::fabs(desired_vel.x())*dt; //TODO: fix this logic: what does vel actually mean here?
            // TODO: in Kota's code, vel is a double -- what does that mean?
            goal.p.y = current_pos.y() + s*std::fabs(desired_vel.y())*dt; // the sign is already baked into the separation displacement variables, so take fabs to get speed rather than velocity
            RCLCPP_INFO(
            this->get_logger(),
            "goal x value (%.2f) , goal y (%.2f), desired x (%.2f), desired y (%.2f)",
            goal.p.x, goal.p.y, dest_pos.p.x, dest_pos.p.y);
            // make the vel ref smooth
            // old lines are:
                //goal.v.x = c*vel;
                //goal.v.y = s*vel;
            RCLCPP_INFO(
            this->get_logger(),
            "c (%.2f) , s (%.2f)",
            c, s);
            goal.v.x = std::min(current_vel.x() + accel_for_vel*dt, c*desired_vel.x()); // this portion no longer makes sense - c and s extract out components of total velocity
            goal.v.y = std::min(current_vel.y() + accel_for_vel*dt, s*desired_vel.y());
            RCLCPP_INFO(
            this->get_logger(),
            "vx (%.2f) , vy (%.2f)",
            goal.v.x, goal.v.y);
            // RCLCPP_INFO(
            // this->get_logger(),
            // "x_vel value (%.2f) , y_vel (%.2f)",
            // goal.v.x, goal.v.y);
            // RCLCPP_INFO(
            // this->get_logger(),
            // "c value (%.2f) , s (%.2f)",
            // c, s);
        }else{
            goal.p.x = dest_pos.p.x;
            goal.p.y = dest_pos.p.y;

            // make the vel ref smooth
            // old lines are:
                //goal.v.x = 0;
                //goal.v.y = 0;
            
            goal.v.x = std::max(0.0, current_vel.x() - accel_for_vel*dt);
            goal.v.y = std::max(0.0, current_vel.y() - accel_for_vel*dt);
            
        }
        // is the yaw close enough to the desired?
        if(yaw_far){
            int sgn = delta_yaw >= 0? 1 : -1;
            vel_yaw = sgn*vel_yaw;  // ccw or cw, the smallest angle
            goal.psi = current_psi + vel_yaw*dt;
            goal.dpsi = vel_yaw;
        }else{
            goal.psi = dest_yaw;
            goal.dpsi = 0;
        }

        // Remember to set power
        goal.power = true;

        return goal;
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
        const auto& F_qmsg = follower_odom_.pose.pose.orientation;
        const auto& F_vel = follower_odom_.twist.twist.linear;
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
        Eigen::Quaterniond F_q = quatFromMsg(F_qmsg);

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
        Eigen::Vector3d follower_vel_tnb(F_vel.x, F_vel.y, F_vel.z);
        Eigen::Vector3d follower_vel_world = F_q * follower_vel_tnb;
        // Eigen::Vector3d leader_vel_tnb = L_q.inverse() * leader_vel_world; // use this if the linear velocity message was in map frame

        double sT = applyDeadband(e_form.x(), deadband_);
        double sN = applyDeadband(e_form.y(), deadband_);
        double sB = applyDeadband(e_form.z(), deadband_);
        double uT = leader_vel_tnb.x() + u_follower_max_ * sT / std::sqrt(sT*sT + p_tuning_ * p_tuning_);
        double uN = leader_vel_tnb.y() + u_follower_max_ * sN / std::sqrt(sN*sN + p_tuning_ * p_tuning_);
        double uB = leader_vel_tnb.z() + u_follower_max_ * sB / std::sqrt(sB*sB + p_tuning_ * p_tuning_);

        Eigen::Vector3d v_TNB(uT,uN,uB);
        double norm = v_TNB.norm();
        norm = std::fabs(norm);
        if (norm > u_follower_max_ && norm  > 1e-9) {
            if(norm > 1) {
                v_TNB *= (u_follower_max_ / norm);
            }
            else {
                 v_TNB *= (u_follower_max_ * norm);
            }
        }
        // Get velocity back into world frame
        Eigen::Vector3d v_world_desired = L_q * v_TNB;

        snapstack_msgs2::msg::Goal goal;
        goal.header.stamp = this->now();

        // position target
        goal.p.x = desired_p_follower.x(); 
        goal.p.y = desired_p_follower.y(); 
        goal.p.z = desired_p_follower.z(); 

        // accel / jerk can be zero for now

        goal.v.x = v_world_desired.x();
        goal.v.y = v_world_desired.y();
        goal.v.z = v_world_desired.z();

        goal.a.x = 0.0;
        goal.a.y = 0.0;
        goal.a.z = 0.0;

        goal.j.x = 0.0;
        goal.j.y = 0.0;
        goal.j.z = 0.0;

        // same yaw as leader
        goal.psi = quat2yaw(L_qmsg);
        goal.dpsi = 0.0;
        goal.mode_xy = snapstack_msgs2::msg::Goal::MODE_VELOCITY_CONTROL;
        goal.mode_z = snapstack_msgs2::msg::Goal::MODE_VELOCITY_CONTROL;
        goal.power = true;
        bool finished; 
        
        snapstack_msgs2::msg::Goal goal_int;
        double desired_const_vel = 0.4;
 

        goal_pub_->publish(goal);
        RCLCPP_INFO(
            this->get_logger(),
            "velocities (%.2f, %.2f, %.2f) -> norm (%.2f)",
            goal.v.x, goal.v.y, goal.v.z,
            v_TNB.norm());
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

    double wrap(double val)
    {
    if(val > M_PI)
        val -= 2.0*M_PI;
    if(val < -M_PI)
        val += 2.0*M_PI;
    return val;
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
    double dist_threshold_{0.5};
    double yaw_threshold_{0.5};

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerGoalGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
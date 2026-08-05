#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <snapstack_msgs2/msg/goal.hpp>
#include "snapstack_msgs2/msg/quad_flight_mode.hpp"

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
        this->declare_parameter<double>("tuning_p", 0.01);
        this->declare_parameter<double>("tuning_d", 0.01);
        this->declare_parameter<double>("max_speed", 2.0);
        this->declare_parameter<double>("deadzone_vctrl", 0.05);
        this->declare_parameter<std::string>("follower_topic", "mavros/local_position/odom");
        this->declare_parameter("margin_takeoff_outside_bounds", 0.0);
        this->declare_parameter("x_min", 0.0);
        this->declare_parameter("x_max", 0.0);
        this->declare_parameter("y_min", 0.0);
        this->declare_parameter("y_max", 0.0);
        this->declare_parameter("z_min", 0.0);
        this->declare_parameter("z_max", 0.0);
        this->declare_parameter("vel_initpos", 0.0);
        this->declare_parameter("vel_take", 0.0);
        this->declare_parameter("vel_land_fast", 0.0);
        this->declare_parameter("vel_land_slow", 0.0);
        this->declare_parameter("vel_yaw", 0.0);
        this->declare_parameter("alt", 0.0);
        this->declare_parameter<double>("cbf_dmin", 0.0);
        this->declare_parameter<double>("cbf_dmax", 0.0);
        this->declare_parameter<double>("cbf_alpha", 0.0);
        this->declare_parameter<bool>("use_cbf", false);
        // Initial position of follower drone with respect to map frame
        this->declare_parameter<std::vector<double>>("init_follower_offset", {0.0, 3.0, 0.0}); 
        this->get_parameter("follower_mode", follower_mode_);
        this->get_parameter("follower_distance_T", follower_distance_T_);
        this->get_parameter("follower_distance_N", follower_distance_N_);
        this->get_parameter("follower_distance_B", follower_distance_B_);
        this->get_parameter("leader_topic", leader_topic_);
        this->get_parameter("goal_altitude", goal_altitude_);
        this->get_parameter("init_follower_offset", init_follower_offset_);
        this->get_parameter("tuning_p", p_tuning_);
        this->get_parameter("tuning_d", d_tuning_);
        this->get_parameter("max_speed", u_follower_max_);
        this->get_parameter("deadzone_vctrl", deadband_);
        this->get_parameter("follower_topic", follower_topic_);
        // Make sure to have environment bounds
        this->get_parameter("x_min", xmin_);
        this->get_parameter("x_max", xmax_);
        this->get_parameter("y_min", ymin_);
        this->get_parameter("y_max", ymax_);
        this->get_parameter("z_min", zmin_);
        this->get_parameter("z_max", zmax_);
        this->get_parameter("vel_initpos", vel_initpos_);
        this->get_parameter("vel_take", vel_take_);
        this->get_parameter("vel_land_fast", vel_land_fast_);
        this->get_parameter("vel_land_slow", vel_land_slow_);
        this->get_parameter("vel_yaw", vel_yaw_);
        this->get_parameter("alt", alt_);
        this->get_parameter("cbf_dmin", d_min_);
        this->get_parameter("cbf_dmax", d_max_);
        this->get_parameter("cbf_alpha", alpha_);
        this->get_parameter("use_cbf", use_cbf_);
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
            follower_topic_, 
            qos_profile,
            std::bind(&FollowerGoalGenerator::followerCB, this, _1));

        mode_sub_ = this->create_subscription<snapstack_msgs2::msg::QuadFlightMode>("globalflightmode", 1, std::bind(&FollowerGoalGenerator::modeCB, this, _1));

        goal_pub_ = this->create_publisher<snapstack_msgs2::msg::Goal>("goal", 10);

        debug_sep_distance_ = this->create_publisher<geometry_msgs::msg::Vector3>("separation_dist", 10);

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

    void modeCB(const snapstack_msgs2::msg::QuadFlightMode& msg){
        // FSM transitions:
        // Any state        --ESTOP--> [kill motors] and switch to ground mode
        // On the ground    --START--> Take off and then hover
        // Hovering         --START--> Go to init pos of the trajectory
        // Init pos of traj --START--> follow the generated trajectory
        // Init pos of traj --END--> switch to hovering where the drone currently is
        // Traj following   --END--> change the traj goals vector to a braking trajectory and then switch to hovering
        // Hovering         --END--> Go to the initial position, land, and then switch to ground mode

        // Behavior selector button to mode mapping
        // START -> GO   (4)
        // END   -> LAND (2)
        // ESTOP -> KILL (6)
        if(msg.mode == msg.KILL){
            goal_.power = false;
            goal_.header.stamp = this->now();
            goal_pub_->publish(goal_);
            flight_mode_ = GROUND;
            resetGoal();
            RCLCPP_INFO(this->get_logger(), "Motors killed, switched to GROUND mode.");
            return;
        }
        else if(flight_mode_ == GROUND and msg.mode == msg.GO){
            // Check inside safety bounds, and don't let the takeoff happen if outside them
            double xmin = xmin_ - margin_takeoff_outside_bounds_;
            double ymin = ymin_ - margin_takeoff_outside_bounds_;
            double xmax = xmax_ + margin_takeoff_outside_bounds_;
            double ymax = ymax_ + margin_takeoff_outside_bounds_;
            if(follower_odom_.pose.pose.position.x < xmin or follower_odom_.pose.pose.position.x > xmax or
            follower_odom_.pose.pose.position.y < ymin or follower_odom_.pose.pose.position.y > ymax){
                RCLCPP_WARN(this->get_logger(), "Can't take off: the vehicle is outside the safety bounds.");
                return;
            }

            // Takeoff initializations
            init_pos_.x = follower_odom_.pose.pose.position.x;
            init_pos_.y = follower_odom_.pose.pose.position.y;
            init_pos_.z = follower_odom_.pose.pose.position.z;
            // set the goal to our current position + yaw
            resetGoal();
            goal_.p.x = init_pos_.x;
            goal_.p.y = init_pos_.y;
            goal_.p.z = init_pos_.z;
            goal_.psi = quat2yaw(follower_odom_.pose.pose.orientation);

            // Take off
            flight_mode_ = TAKING_OFF;
            RCLCPP_INFO(this->get_logger(), "Taking off...");
            // then it will switch automatically to HOVERING

            // switch on motors after flight_mode changes, to avoid timer callback setting power to false
            goal_.power = true;
        }
        else if(flight_mode_ == HOVERING and msg.mode == msg.GO){
            flight_mode_ = INIT_POS_TRAJ;
            RCLCPP_INFO(this->get_logger(), "Going to the initial position of the generated trajectory...");
        }
        else if(flight_mode_ == INIT_POS_TRAJ and msg.mode == msg.GO){
            followerLogic(); 
            flight_mode_ = TRAJ_FOLLOWING;
            RCLCPP_INFO(this->get_logger(), "Following the trajectory...");
        }
        else if(flight_mode_ == INIT_POS_TRAJ and msg.mode == msg.LAND){
            // Change mode to hover wherever the robot was when we clicked "END"
            // Need to send a current goal with 0 vel bc we could be moving to the init pos of traj
            resetGoal();
            goal_.p.x = follower_odom_.pose.pose.position.x;
            goal_.p.y = follower_odom_.pose.pose.position.y;
            goal_.p.z = follower_odom_.pose.pose.position.z;
            goal_.psi = quat2yaw(follower_odom_.pose.pose.orientation);
            goal_.header.stamp = this->now();
            goal_pub_->publish(goal_);
            flight_mode_ = HOVERING;
            RCLCPP_INFO(this->get_logger(), "Switched to HOVERING mode");
        }
        else if(flight_mode_ == TRAJ_FOLLOWING and msg.mode == msg.LAND){
            // Hover
            // Change mode to hover wherever the robot was when we clicked "END"
            // Need to send a current goal with 0 vel bc we could be moving to the init pos of traj
            resetGoal();
            goal_.p.x = follower_odom_.pose.pose.position.x;
            goal_.p.y = follower_odom_.pose.pose.position.y;
            goal_.p.z = follower_odom_.pose.pose.position.z;
            goal_.psi = quat2yaw(follower_odom_.pose.pose.orientation);
            goal_.header.stamp = this->now();
            goal_pub_->publish(goal_);
            flight_mode_ = HOVERING;
            RCLCPP_INFO(this->get_logger(), "Switched to HOVERING mode");
        }
        else if(flight_mode_ == HOVERING and msg.mode == msg.LAND){
            //go to the initial position
            flight_mode_ = INIT_POS;
            RCLCPP_INFO(this->get_logger(), "Switched to INIT_POS mode");
        }
    }


    snapstack_msgs2::msg::Goal simpleInterpolation(const snapstack_msgs2::msg::Goal& current,
            const geometry_msgs::msg::Vector3& dest_pos, double dest_yaw, double vel, double vel_yaw,
            double dist_thresh, double yaw_thresh, double dt, bool& finished){
        snapstack_msgs2::msg::Goal goal;
        // interpolate from current goal pos to the initial goal pos
        double Dx = dest_pos.x - current.p.x;
        double Dy = dest_pos.y - current.p.y;
        double dist = sqrt(Dx*Dx + Dy*Dy);

        double delta_yaw = dest_yaw - current.psi;
        delta_yaw = wrap(delta_yaw);

        bool dist_far = dist > dist_thresh;
        bool yaw_far  = fabs(delta_yaw) > yaw_thresh;
        finished = not dist_far and not yaw_far;  // both are close

        bool accel_for_vel = 0.1;

        goal.p.z = dest_pos.z;  // this should be alt_ and the altitude where the drone took off too
        // are we too far from the dest?
        if(dist_far){
            double c = Dx/dist;
            double s = Dy/dist;
            goal.p.x = current.p.x + c*vel*dt;
            goal.p.y = current.p.y + s*vel*dt;
            
            // make the vel ref smooth
            // old lines are:
                //goal.v.x = c*vel;
                //goal.v.y = s*vel;
            
            goal.v.x = std::min(current.v.x + accel_for_vel*dt, c*vel);
            goal.v.y = std::min(current.v.y + accel_for_vel*dt, s*vel);

        }else{
            goal.p.x = dest_pos.x;
            goal.p.y = dest_pos.y;

            // make the vel ref smooth
            // old lines are:
                //goal.v.x = 0;
                //goal.v.y = 0;
            
            goal.v.x = std::max(0.0, current.v.x - accel_for_vel*dt);
            goal.v.y = std::max(0.0, current.v.y - accel_for_vel*dt);
            
        }
        // is the yaw close enough to the desired?
        if(yaw_far){
            int sgn = delta_yaw >= 0? 1 : -1;
            vel_yaw = sgn*vel_yaw;  // ccw or cw, the smallest angle
            goal.psi = current.psi + vel_yaw*dt;
            goal.dpsi = vel_yaw;
        }else{
            goal.psi = dest_yaw;
            goal.dpsi = 0;
        }

        // Remember to set power
        goal.power = true;

        return goal;
    }


    void followerLogic(){

        // Get positions & orientations
        const auto& L_pos = leader_odom_.pose.pose.position;
        const auto& L_qmsg = leader_odom_.pose.pose.orientation;
        const auto& L_vel = leader_odom_.twist.twist.linear;

        const auto& F_pos = follower_odom_.pose.pose.position;
        const auto& F_qmsg = follower_odom_.pose.pose.orientation;
        const auto& F_vel = follower_odom_.twist.twist.linear;

        Eigen::Quaterniond L_q = quatFromMsg(L_qmsg);
        Eigen::Quaterniond F_q = quatFromMsg(F_qmsg);

        // Leader/follower positions (note here map and world are the same - TODO: go back and use consistent wording)
        Eigen::Vector3d leader_pos_world(L_pos.x, L_pos.y, L_pos.z);
        Eigen::Vector3d follower_pos_local(F_pos.x, F_pos.y, F_pos.z);

        // Desired offset in leader body frame
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

        // Desired position of the follower in the world frame
        Eigen::Vector3d desired_p_follower = leader_pos_world + L_q * desired_offset_tnb - init_offset_world;

        // Actual position of follower in the world frame
        Eigen::Vector3d follower_pos_world = follower_pos_local + init_offset_world;
        // Relative displacement from follower to leader in world frame
        Eigen::Vector3d rel_world = follower_pos_world - leader_pos_world;
        // Express relative displacement in leader body frame (TNB)
        Eigen::Vector3d rel_tnb = L_q.inverse() * rel_world;

        // Formation error in leader frame
        Eigen::Vector3d e_form = desired_offset_tnb - rel_tnb;

        // DEBUGGING ---------------------------------
        geometry_msgs::msg::Vector3 e_msg;
        e_msg.x = e_form.x();
        e_msg.y = e_form.y();
        e_msg.z = e_form.z();
        debug_sep_distance_->publish(e_msg);
        // ------------------------------------------

        // Leader & follower velocity (note that odom twist message is expressed in child_frame_id)
        Eigen::Vector3d leader_vel_tnb(L_vel.x, L_vel.y, L_vel.z);
        Eigen::Vector3d follower_vel_tnb(F_vel.x, F_vel.y, F_vel.z);
        Eigen::Vector3d follower_vel_world = F_q * follower_vel_tnb;
        // Eigen::Vector3d leader_vel_tnb = L_q.inverse() * leader_vel_world; // use this if the linear velocity message was in map frame

        // Get separation distances and velocity law
        double sT = applyDeadband(e_form.x(), deadband_);
        double sN = applyDeadband(e_form.y(), deadband_);
        double sB = applyDeadband(e_form.z(), deadband_);
        double uT = leader_vel_tnb.x() + u_follower_max_ * sT / std::sqrt(sT*sT + p_tuning_ * p_tuning_); 
        double uN = leader_vel_tnb.y() + u_follower_max_ * sN / std::sqrt(sN*sN + p_tuning_ * p_tuning_);
        double uB = leader_vel_tnb.z() + u_follower_max_ * sB / std::sqrt(sB*sB + p_tuning_ * p_tuning_); 

        if (use_cbf_){

            // Set nominal velocities to 0. 
            // Because in this demo, we are showing how CBFs can be used for formation flight without a nominal input
            uT = 0.0;
            uN = 0.0;
            uB = 0.0;
            Eigen::Vector3d follower_u_tnb(uT, uN, uB); // the nominal inputs
            // Let the moving barrier constraint be projected onto the desired follower positions
            Eigen::Vector3d vel_err_tnb = follower_u_tnb - leader_vel_tnb;
            Eigen::Vector3d relative_sep = - e_form; // relative separation between the desired and actual follower position (x_F - (x_L + desired_offset))
            // Compute lagrangian multipliers
            double delH1_x = vel_err_tnb.x() + alpha_*(relative_sep.x() - d_max_);
            double delH2_x = -vel_err_tnb.x() + alpha_*(-relative_sep.x() - d_max_);
            double delH1_y = vel_err_tnb.y() + alpha_*(relative_sep.y() - d_max_);
            double delH2_y = -vel_err_tnb.y() + alpha_*(-relative_sep.y() - d_max_);
            double delH1_z = vel_err_tnb.z() + alpha_*(relative_sep.z() - d_max_);
            double delH2_z = -vel_err_tnb.z() + alpha_*(-relative_sep.z() - d_max_);
            double lambda1_x = 2*std::max(0.0, delH1_x);
            double lambda2_x = 2*std::max(0.0, delH2_x);
            double lambda1_y = 2*std::max(0.0, delH1_y);
            double lambda2_y = 2*std::max(0.0, delH2_y);
            double lambda1_z = 2*std::max(0.0, delH1_z);
            double lambda2_z = 2*std::max(0.0, delH2_z);
            // Compute the augmentation.
            double pi_x = 0.5 * (lambda2_x - lambda1_x);
            double pi_y = 0.5 * (lambda2_y - lambda1_y);
            double pi_z = 0.5 * (lambda2_z - lambda1_z);

            uT += pi_x;
            uN += pi_y;
            uB += pi_z;
            RCLCPP_INFO(
            this->get_logger(),
            "current pos error (%.2f, %.2f %.2f), velocities (%.2f, %.2f, %.2f)",
            std::abs(e_form.x()), std::abs(e_form.y()), std::abs(e_form.z()), uT, uN, uB);
        }

        // Saturate velocity commands that exceed max_speed
        Eigen::Vector3d v_TNB(uT,uN,uB);
        double norm = v_TNB.norm();
        norm = std::fabs(norm);
        if (norm > u_follower_max_ && norm  > 1e-9) {
            RCLCPP_INFO(this->get_logger(), "SATURATED");
            v_TNB *= (u_follower_max_ / norm);
        }

        // Get velocity back into world frame
        Eigen::Vector3d v_world_desired = L_q * v_TNB;

        // Update Goal
        resetGoal();
        goal_.header.stamp = this->now();

        // Position target
        goal_.p.x = NAN; // want position to be NAN so we can send velocity control commands 
        goal_.p.y = NAN; 
        goal_.p.z = NAN; 

        // accel / jerk can be zero for now

        goal_.v.x = v_world_desired.x();
        goal_.v.y = v_world_desired.y();
        goal_.v.z = v_world_desired.z();

        goal_.a.x = 0.0;
        goal_.a.y = 0.0;
        goal_.a.z = 0.0;

        goal_.j.x = 0.0;
        goal_.j.y = 0.0;
        goal_.j.z = 0.0;

        // same yaw as leader
        goal_.psi = quat2yaw(L_qmsg);
        goal_.dpsi = 0.0;
        goal_.mode_xy = snapstack_msgs2::msg::Goal::MODE_VELOCITY_CONTROL;
        goal_.mode_z = snapstack_msgs2::msg::Goal::MODE_VELOCITY_CONTROL;
        goal_.power = true;
        bool finished; 

    }

    // Publish the goal for the follower
    // NOTE: ANYWHERE THAT GOAL POSITIONS ARE NOT NAN, PX4 WILL DEFAULT TO POSITION CONTROL
    void pubCB()
    {
        if (!has_leader_ || !has_follower_) {
            return;
        }

        // Always publish a goal to avoid ramps in comm_monitor
        if(flight_mode_ == GROUND)
            goal_.power = false;  // not needed but just in case, for safety

        // if taking off, increase alt until we reach
        else if(flight_mode_ == TAKING_OFF){
            // TODO: spinup time

            double takeoff_alt = alt_;  // don't add init alt bc the traj is generated with z = alt_
            double eps = 0.10; //TODO: Change back to 0.10
            // if close to the takeoff_alt, switch to HOVERING
            // RCLCPP_INFO(this->get_logger(), "Takeoff alt: %f", alt_);
            RCLCPP_INFO(this->get_logger(), "Pose z: %f", follower_odom_.pose.pose.position.z);
            if(fabs(takeoff_alt - follower_odom_.pose.pose.position.z) < eps and goal_.p.z >= takeoff_alt){
                flight_mode_ = HOVERING;
                RCLCPP_INFO(this->get_logger(), "Take off completed");
            }
            else{
                // Increment the z cmd each timestep for a smooth takeoff.
                // This is essentially saturating tracking error so actuation is low.
                goal_.p.z = saturate(goal_.p.z + vel_take_*dt_, 0.0, takeoff_alt);
            }
        }
        //else if(flight_mode_ == HOVERING) <- just publish current goal
        else if(flight_mode_ == INIT_POS_TRAJ){
            // Since the follower doesn't have a specific init_pos of a trajectory, do nothing in this phase
            // If desired, maybe change behavior to go back to starting position of takeoff
        }
        else if(flight_mode_ == TRAJ_FOLLOWING){
            followerLogic();
        }
        else if(flight_mode_ == INIT_POS){
            // go to init_pos_ but with altitude alt_ and current yaw
            bool finished;
            geometry_msgs::msg::Vector3 dest = init_pos_;
            dest.z = alt_;
            goal_ = simpleInterpolation(goal_, dest, goal_.psi, vel_initpos_, vel_yaw_,
                                        dist_thresh_, yaw_thresh_, dt_, finished);

            if(finished){ // land when close to the init pos
                flight_mode_ = LANDING;
                RCLCPP_INFO(this->get_logger(), "Landing...");
            }
        }
        // if landing, decrease alt until we reach ground (and switch to ground)
        // The goal was already set to our current position + yaw when hovering
        else if(flight_mode_ == LANDING){
            // choose between fast and slow landing
            double vel_land = follower_odom_.pose.pose.position.z > (init_pos_.z + 0.4)? vel_land_fast_ : vel_land_slow_;
            goal_.p.z = goal_.p.z - vel_land*dt_;

            if(goal_.p.z < 0){  // don't use init alt here. It's safer to try to land to the ground
                // landed, kill motors
                goal_.power = false;
                flight_mode_ = GROUND;
                RCLCPP_INFO(this->get_logger(), "Landed");
            }
            goal_.mode_xy = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
            goal_.mode_z = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
        }

        // apply safety bounds
        goal_.p.x = saturate(goal_.p.x, xmin_, xmax_);  // val, low, high
        goal_.p.y = saturate(goal_.p.y, ymin_, ymax_);
        goal_.p.z = saturate(goal_.p.z, zmin_, zmax_);

        goal_.header.stamp = this->now(); // set current time

        // Goals should only be published here because this is the only place where we
        // apply safety bounds. Exceptions: when killing the drone and when clicking END at init pos traj

        goal_pub_->publish(goal_);
    }

    void resetGoal(){
        // Creating a new goal message should already set this correctly, but just in case
        // Exception: yaw would be 0 instead of current yaw
        goal_.p.x = goal_.p.y = goal_.p.z = 0;
        goal_.v.x = goal_.v.y = goal_.v.z = 0;
        goal_.a.x = goal_.a.y = goal_.a.z = 0;
        goal_.j.x = goal_.j.y = goal_.j.z = 0;
        //goal_.s.x = goal_.s.y = goal_.s.z = 0;
        goal_.psi = quat2yaw(follower_odom_.pose.pose.orientation); goal_.dpsi = 0;
    //    goal_.power = false;
        // reset_xy_int and  reset_z_int are not used
        goal_.mode_xy = snapstack_msgs2::msg::Goal::MODE_VELOCITY_CONTROL;
        goal_.mode_z = snapstack_msgs2::msg::Goal::MODE_VELOCITY_CONTROL;
    }


    double quat2yaw(const geometry_msgs::msg::Quaternion& q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    double saturate(double val, double low, double high){
        if(val > high)
            val = high;
        else if(val < low)
            val = low;

        return val;
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
    rclcpp::Subscription<snapstack_msgs2::msg::QuadFlightMode>::SharedPtr mode_sub_;  // "flightmode" Subscription
    rclcpp::Publisher<snapstack_msgs2::msg::Goal>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr debug_sep_distance_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry leader_odom_;
    nav_msgs::msg::Odometry follower_odom_;
    bool has_leader_{false};
    bool has_follower_{false};
    snapstack_msgs2::msg::Goal goal_;


    // for velocity commands
    double u_follower_max_; // max speed m/s which follower can approach leader
    double L_T_vel_;
    double L_N_vel_;
    double L_B_vel_;

    // For mode callback
    enum FlightMode {GROUND,
                    TAKING_OFF,
                    HOVERING,
                    INIT_POS_TRAJ,
                    TRAJ_FOLLOWING,
                    LANDING,
                    INIT_POS
                };
    FlightMode flight_mode_;
    double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_;  // safety bouds
    double vel_initpos_, vel_take_, vel_land_fast_, vel_land_slow_, vel_yaw_;  // interpolation vels
    double margin_takeoff_outside_bounds_;
    geometry_msgs::msg::Vector3 init_pos_;
    double alt_;  // altitude in m where to take off

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
    double d_tuning_;
    double deadband_;
    double dist_thresh_{0.5};
    double yaw_thresh_{0.5};

    // FOR CBF
    double d_min_;
    double d_max_;
    double alpha_;
    bool use_cbf_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerGoalGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
/**
 * @file TrajectoryGenerator.cpp
 * @brief Trajectory Generator class
 * @author Aleix Paris
 * @date 2020-01-08
 */

#include "trajectory_generator_ros2/TrajectoryGenerator.hpp"
#include "trajectory_generator_ros2/trajectories/Circle.hpp"
#include "trajectory_generator_ros2/trajectories/Line.hpp"
#include "trajectory_generator_ros2/trajectories/Boomerang.hpp"
#include "trajectory_generator_ros2/trajectories/Figure8.hpp"
#include "trajectory_generator_ros2/trajectories/Square.hpp"
#include "trajectory_generator_ros2/trajectories/Reciprocating.hpp"
#include "trajectory_generator_ros2/trajectories/Rectangle.hpp"
#include "trajectory_generator_ros2/trajectories/Bounce.hpp"
#include "trajectory_generator_ros2/trajectories/M.hpp"
#include "trajectory_generator_ros2/trajectories/I.hpp"
#include "trajectory_generator_ros2/trajectories/T.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "snapstack_msgs2/msg/quad_flight_mode.hpp"
#include "snapstack_msgs2/msg/state.hpp"
#include "snapstack_msgs2/msg/goal.hpp"

#include <Eigen/Core>
#include <vector>
#include <unordered_map>
#include <string>

#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

/*
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <eigen3/Eigen/Eigen>*/

namespace trajectory_generator {

TrajectoryGenerator::TrajectoryGenerator()
    : Node("trajectory_generator")
{

    //QoS Profile 
    rclcpp::QoS qos_profile(10);
    qos_profile
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .reliability(rclcpp::ReliabilityPolicy::BestEffort);

    if (!readParameters()) {
        RCLCPP_ERROR(this->get_logger(), "Could not read parameters.");
        rclcpp::shutdown();
    }

    // Check namespace to find name of vehicle
    veh_name_ = this->get_namespace();
    size_t n = veh_name_.find_first_not_of('/');
    veh_name_.erase(0, n); // remove leading slashes
    if (veh_name_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Error :: You should be using a launch file to specify the "
                "node namespace!\n");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "veh_name_ = %s", veh_name_.c_str());

    traj_->generateTraj(traj_goals_full_, index_msgs_full_, this->get_clock());
    traj_goals_ = traj_goals_full_;
    index_msgs_ = index_msgs_full_;

    subs_mode_ = this->create_subscription<snapstack_msgs2::msg::QuadFlightMode>("/globalflightmode", 1, std::bind(&TrajectoryGenerator::modeCB, this, _1));
    subs_state_ = this->create_subscription<snapstack_msgs2::msg::State>("state", qos_profile, std::bind(&TrajectoryGenerator::stateCB, this, _1));
    pub_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_), 
        std::bind(&TrajectoryGenerator::pubCB, this));
    pub_goal_  = this->create_publisher<snapstack_msgs2::msg::Goal>("goal", 1);  // topic, queue_size

    rclcpp::sleep_for(std::chrono::seconds(1));  // to ensure that the state has been received

    flight_mode_ = GROUND;

    // Init goal (and update with current pos every time that the mode switches to GROUND)
    resetGoal();
    goal_.p.x = pose_.position.x;
    goal_.p.y = pose_.position.y;
    goal_.p.z = pose_.position.z;

    RCLCPP_INFO(this->get_logger(), "Successfully launched trajectory generator node.");
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}

bool TrajectoryGenerator::readParameters()
{
    // Shared params (flat at the top of the YAML).
    this->declare_parameter("alt", 0.0);
    this->declare_parameter("pub_freq", 0.0);
    this->declare_parameter("traj_type", "");
    this->declare_parameter("vel_initpos", 0.0);
    this->declare_parameter("vel_take", 0.0);
    this->declare_parameter("vel_land_fast", 0.0);
    this->declare_parameter("vel_land_slow", 0.0);
    this->declare_parameter("vel_yaw", 0.0);
    this->declare_parameter("dist_thresh", 0.0);
    this->declare_parameter("yaw_thresh", 0.0);
    this->declare_parameter("margin_takeoff_outside_bounds", 0.0);
    this->declare_parameter("x_min", 0.0);
    this->declare_parameter("x_max", 0.0);
    this->declare_parameter("y_min", 0.0);
    this->declare_parameter("y_max", 0.0);
    this->declare_parameter("z_min", 0.0);
    this->declare_parameter("z_max", 0.0);

    if (!this->get_parameter("alt", alt_)) return false;
    double freq;
    if (!this->get_parameter("pub_freq", freq)) return false;
    dt_ = 1.0 / freq;

    std::string traj_type;
    if (!this->get_parameter("traj_type", traj_type)) return false;

    // Per-trajectory params live in YAML under a block keyed by traj_type,
    // e.g. Figure8.r, Square.side_length, Line.Ax, etc. Only the selected
    // block is declared/read so the YAML can carry unused defaults for
    // trajectories we aren't flying this run.
    const std::string p = traj_type + ".";

    auto declare = [&](const std::string& key, double def) {
        this->declare_parameter(p + key, def);
    };
    auto declare_v = [&](const std::string& key) {
        this->declare_parameter<std::vector<double>>(p + key, {});
    };
    auto get_d = [&](const std::string& key, double& out) -> bool {
        return this->get_parameter(p + key, out);
    };
    auto get_v = [&](const std::string& key, std::vector<double>& out) -> bool {
        return this->get_parameter(p + key, out);
    };
    auto check_v_goals = [&](const std::vector<double>& v) -> bool {
        for (double vel : v) {
            if (vel <= 0) {
                RCLCPP_ERROR(this->get_logger(), "All velocities must be > 0");
                return false;
            }
        }
        return true;
    };

    if (traj_type == "Circle" || traj_type == "Figure8") {
        declare("r", 0.0);
        declare("center_x", 0.0);
        declare("center_y", 0.0);
        declare_v("v_goals");
        declare("t_traj", 0.0);
        declare("circle_accel", 0.0);

        double r, cx, cy, t_traj, circle_accel;
        std::vector<double> v_goals;
        if (!get_d("r", r)) return false;
        if (!get_d("center_x", cx)) return false;
        if (!get_d("center_y", cy)) return false;
        if (!get_v("v_goals", v_goals)) return false;
        if (!check_v_goals(v_goals)) return false;
        if (!get_d("t_traj", t_traj)) return false;
        if (!get_d("circle_accel", circle_accel)) return false;
        if (circle_accel <= 0) { RCLCPP_ERROR(this->get_logger(), "accel must be > 0"); return false; }

        if (traj_type == "Circle")
            traj_ = std::make_unique<Circle>(alt_, r, cx, cy, v_goals, t_traj, circle_accel, dt_);
        else
            traj_ = std::make_unique<Figure8>(alt_, r, cx, cy, v_goals, t_traj, circle_accel, dt_);
    }
    else if (traj_type == "Square" || traj_type == "Rectangle") {
        declare("center_x", 0.0);
        declare("center_y", 0.0);
        declare("orientation", 0.0);
        declare_v("v_goals");
        declare("t_traj", 0.0);

        double cx, cy, orientation, t_traj;
        std::vector<double> v_goals;
        if (!get_d("center_x", cx)) return false;
        if (!get_d("center_y", cy)) return false;
        if (!get_d("orientation", orientation)) return false;
        if (!get_v("v_goals", v_goals)) return false;
        if (!check_v_goals(v_goals)) return false;
        if (!get_d("t_traj", t_traj)) return false;

        if (traj_type == "Square") {
            declare("side_length", 0.0);
            declare("square_accel", 0.0);
            double side_length, square_accel;
            if (!get_d("side_length", side_length)) return false;
            if (!get_d("square_accel", square_accel)) return false;
            if (square_accel <= 0) { RCLCPP_ERROR(this->get_logger(), "accel must be > 0"); return false; }
            traj_ = std::make_unique<Square>(alt_, side_length, cx, cy, orientation, v_goals, t_traj, square_accel, dt_);
        } else {
            declare("side_a", 0.0);
            declare("side_b", 0.0);
            declare("rectangle_accel", 0.0);
            double side_a, side_b, rectangle_accel;
            if (!get_d("side_a", side_a)) return false;
            if (!get_d("side_b", side_b)) return false;
            if (!get_d("rectangle_accel", rectangle_accel)) return false;
            if (rectangle_accel <= 0) { RCLCPP_ERROR(this->get_logger(), "accel must be > 0"); return false; }
            traj_ = std::make_unique<Rectangle>(alt_, side_a, side_b, cx, cy, orientation, v_goals, t_traj, rectangle_accel, dt_);
        }
    }
    else if (traj_type == "Line" || traj_type == "Boomerang" || traj_type == "Reciprocating") {
        declare("Ax", 0.0);
        declare("Ay", 0.0);
        declare("Bx", 0.0);
        declare("By", 0.0);
        declare("v_line", 0.0);
        declare("line_accel", 0.0);
        declare("line_decel", 0.0);

        double Ax, Ay, Bx, By, v_line, a1, a3;
        if (!get_d("Ax", Ax)) return false;
        if (!get_d("Ay", Ay)) return false;
        if (!get_d("Bx", Bx)) return false;
        if (!get_d("By", By)) return false;
        if (!get_d("v_line", v_line)) return false;
        if (v_line <= 0) { RCLCPP_ERROR(this->get_logger(), "velocity must be > 0"); return false; }
        if (!get_d("line_accel", a1)) return false;
        if (!get_d("line_decel", a3)) return false;
        if (a1 <= 0 || a3 <= 0) { RCLCPP_ERROR(this->get_logger(), "accel/decel must be > 0"); return false; }

        std::vector<double> v_goals = {v_line};

        if (traj_type == "Line") {
            traj_ = std::make_unique<Line>(alt_, Eigen::Vector3d(Ax, Ay, alt_),
                                           Eigen::Vector3d(Bx, By, alt_),
                                           v_goals, a1, a3, dt_);
        } else if (traj_type == "Boomerang") {
            traj_ = std::make_unique<Boomerang>(alt_, Eigen::Vector3d(Ax, Ay, alt_),
                                                Eigen::Vector3d(Bx, By, alt_),
                                                v_goals, a1, a3, dt_);
        } else {  // Reciprocating
            declare("t_traj", 0.0);
            double t_traj;
            if (!get_d("t_traj", t_traj)) return false;
            traj_ = std::make_unique<Reciprocating>(alt_, Eigen::Vector3d(Ax, Ay, alt_),
                                                    Eigen::Vector3d(Bx, By, alt_),
                                                    v_goals, a1, a3, t_traj, dt_);
        }
    }
    else if (traj_type == "Bounce") {
        declare("center_x", 0.0);
        declare("center_y", 0.0);
        declare("Az", 0.0);
        declare("Bz", 0.0);
        declare("orientation", 0.0);
        declare_v("v_goals");
        declare("t_traj", 0.0);

        double cx, cy, Az, Bz, orientation, t_traj;
        std::vector<double> v_goals;
        if (!get_d("center_x", cx)) return false;
        if (!get_d("center_y", cy)) return false;
        if (!get_d("Az", Az)) return false;
        if (!get_d("Bz", Bz)) return false;
        if (!get_d("orientation", orientation)) return false;
        if (!get_v("v_goals", v_goals)) return false;
        if (!check_v_goals(v_goals)) return false;
        if (!get_d("t_traj", t_traj)) return false;

        traj_ = std::make_unique<Bounce>(cx, cy, Az, Bz, v_goals, t_traj, orientation, dt_);
    }
    else if (traj_type == "M" || traj_type == "I" || traj_type == "T") {
        const std::string dim = traj_type + "_";  // "M_", "I_", "T_"
        declare("center_x", 0.0);
        declare("center_y", 0.0);
        declare(dim + "length", 0.0);
        declare(dim + "width", 0.0);
        declare("orientation", 0.0);
        declare_v("v_goals");
        declare("t_traj", 0.0);

        double cx, cy, length, width, t_traj, orientation;
        std::vector<double> v_goals;
        if (!get_d("center_x", cx)) return false;
        if (!get_d("center_y", cy)) return false;
        if (!get_d(dim + "length", length)) return false;
        if (!get_d(dim + "width", width)) return false;
        if (!get_v("v_goals", v_goals)) return false;
        if (!check_v_goals(v_goals)) return false;
        if (!get_d("t_traj", t_traj)) return false;
        if (!get_d("orientation", orientation)) return false;

        if (traj_type == "M")
            traj_ = std::make_unique<M>(cx, cy, length, width, alt_, v_goals, t_traj, orientation, dt_);
        else if (traj_type == "I")
            traj_ = std::make_unique<I>(cx, cy, length, width, alt_, v_goals, t_traj, orientation, dt_);
        else
            traj_ = std::make_unique<T>(cx, cy, length, width, alt_, v_goals, t_traj, orientation, dt_);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Trajectory type '%s' not valid.", traj_type.c_str());
        return false;
    }

    // other params
    if (!this->get_parameter("vel_initpos", vel_initpos_)) return false;
    if (!this->get_parameter("vel_take", vel_take_)) return false;
    if (!this->get_parameter("vel_land_fast", vel_land_fast_)) return false;
    if (!this->get_parameter("vel_land_slow", vel_land_slow_)) return false;
    if (!this->get_parameter("vel_yaw", vel_yaw_)) return false;

    if (!this->get_parameter("dist_thresh", dist_thresh_)) return false;
    if (!this->get_parameter("yaw_thresh", yaw_thresh_)) return false;

    if (!this->get_parameter("margin_takeoff_outside_bounds", margin_takeoff_outside_bounds_)) return false;

    // bounds
    // if (!this->get_parameter("/room_bounds/x_min", xmin_)) return false;
    // if (!this->get_parameter("/room_bounds/x_max", xmax_)) return false;
    // if (!this->get_parameter("/room_bounds/y_min", ymin_)) return false;
    // if (!this->get_parameter("/room_bounds/y_max", ymax_)) return false;
    // if (!this->get_parameter("/room_bounds/z_min", zmin_)) return false;
    // if (!this->get_parameter("/room_bounds/z_max", zmax_)) return false;

    //bounds
    if (!this->get_parameter("x_min", xmin_)) return false;
    if (!this->get_parameter("x_max", xmax_)) return false;
    if (!this->get_parameter("y_min", ymin_)) return false;
    if (!this->get_parameter("y_max", ymax_)) return false;
    if (!this->get_parameter("z_min", zmin_)) return false;
    if (!this->get_parameter("z_max", zmax_)) return false;
    // check that the trajectory params don't conflict with the bounds
    
    if(!traj_->trajectoryInsideBounds(xmin_, xmax_, ymin_, ymax_, zmin_, zmax_)){
        RCLCPP_ERROR(this->get_logger(), "TheF trajectory parameters conflict with the room bounds.");
        return false;
    }

    return true;
}

void TrajectoryGenerator::modeCB(const snapstack_msgs2::msg::QuadFlightMode& msg){
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
        pub_goal_->publish(goal_);
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
        if(pose_.position.x < xmin or pose_.position.x > xmax or
           pose_.position.y < ymin or pose_.position.y > ymax){
            RCLCPP_WARN(this->get_logger(), "Can't take off: the vehicle is outside the safety bounds.");
            return;
        }

        // Takeoff initializations
        init_pos_.x = pose_.position.x;
        init_pos_.y = pose_.position.y;
        init_pos_.z = pose_.position.z;
        // set the goal to our current position + yaw
        resetGoal();
        goal_.p.x = init_pos_.x;
        goal_.p.y = init_pos_.y;
        goal_.p.z = init_pos_.z;
        goal_.psi = quat2yaw(pose_.orientation);

        // Take off
        flight_mode_ = TAKING_OFF;
        RCLCPP_INFO(this->get_logger(), "Taking off...");
        // then it will switch automatically to HOVERING

        // switch on motors after flight_mode changes, to avoid timer callback setting power to false
        goal_.power = true;
    }
    else if(flight_mode_ == HOVERING and msg.mode == msg.GO){
        traj_goals_ = traj_goals_full_;
        index_msgs_ = index_msgs_full_;
        flight_mode_ = INIT_POS_TRAJ;
        RCLCPP_INFO(this->get_logger(), "Going to the initial position of the generated trajectory...");
    }
    else if(flight_mode_ == INIT_POS_TRAJ and msg.mode == msg.GO){
        // Start following the generated trajectory if close to the init pos (in 2D)
        double dist_to_init = sqrt(pow(traj_goals_[0].p.x - pose_.position.x, 2) +
                                   pow(traj_goals_[0].p.y - pose_.position.y, 2));
        double delta_yaw = traj_goals_[0].psi - quat2yaw(pose_.orientation);
        delta_yaw = wrap(delta_yaw);
        if(dist_to_init > dist_thresh_ or fabs(delta_yaw) > yaw_thresh_){
            RCLCPP_INFO(this->get_logger(), "Can't switch to the generated trajectory following mode, too far from the init pos");
            return;
        }
        pub_index_ = 0;
        flight_mode_ = TRAJ_FOLLOWING;
        RCLCPP_INFO(this->get_logger(), "Following the generated trajectory...");
    }
    else if(flight_mode_ == INIT_POS_TRAJ and msg.mode == msg.LAND){
        // Change mode to hover wherever the robot was when we clicked "END"
        // Need to send a current goal with 0 vel bc we could be moving to the init pos of traj
        resetGoal();
        goal_.p.x = pose_.position.x;
        goal_.p.y = pose_.position.y;
        goal_.p.z = alt_;
        goal_.psi = quat2yaw(pose_.orientation);
        goal_.header.stamp = this->now();
        pub_goal_->publish(goal_);
        flight_mode_ = HOVERING;
        RCLCPP_INFO(this->get_logger(), "Switched to HOVERING mode");
    }
    else if(flight_mode_ == TRAJ_FOLLOWING and msg.mode == msg.LAND){
        // Generate a braking trajectory. Then, we will automatically switch to hover when done
        traj_->generateStopTraj(traj_goals_, index_msgs_, pub_index_, this->get_clock());
    }
    else if(flight_mode_ == HOVERING and msg.mode == msg.LAND){
        //go to the initial position
        flight_mode_ = INIT_POS;
        RCLCPP_INFO(this->get_logger(), "Switched to INIT_POS mode");
    }
}

void TrajectoryGenerator::pubCB(){
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
        RCLCPP_INFO(this->get_logger(), "Pose z: %f", pose_.position.z);
        if(fabs(takeoff_alt - pose_.position.z) < eps and goal_.p.z >= takeoff_alt){
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
        bool finished;
        goal_ = simpleInterpolation(goal_, traj_goals_[0], traj_goals_[0].psi,
                vel_initpos_, vel_yaw_, dist_thresh_, yaw_thresh_, dt_, finished);
        // if finished, switch to traj following mode? No, prefer to choose when
    }
    else if(flight_mode_ == TRAJ_FOLLOWING){
        goal_ = traj_goals_[pub_index_];
        if(index_msgs_.find(pub_index_) != index_msgs_.end()){
            RCLCPP_INFO(this->get_logger(), "%s", index_msgs_[pub_index_].c_str());
        }
        ++pub_index_;
        if(pub_index_ == traj_goals_.size()){
            // Switch to HOVERING mode after finishing the trajectory
            resetGoal();
            goal_.p.x = pose_.position.x;
            goal_.p.y = pose_.position.y;
            goal_.p.z = alt_;
            goal_.psi = quat2yaw(pose_.orientation);
            pub_goal_->publish(goal_);
            flight_mode_ = HOVERING;
            RCLCPP_INFO(this->get_logger(), "Trajectory finished. Switched to HOVERING mode");
        }
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
        double vel_land = pose_.position.z > (init_pos_.z + 0.4)? vel_land_fast_ : vel_land_slow_;
        goal_.p.z = goal_.p.z - vel_land*dt_;

        if(goal_.p.z < 0){  // don't use init alt here. It's safer to try to land to the ground
            // landed, kill motors
            goal_.power = false;
            flight_mode_ = GROUND;
            RCLCPP_INFO(this->get_logger(), "Landed");
        }
    }

    // apply safety bounds
    goal_.p.x = saturate(goal_.p.x, xmin_, xmax_);  // val, low, high
    goal_.p.y = saturate(goal_.p.y, ymin_, ymax_);
    goal_.p.z = saturate(goal_.p.z, zmin_, zmax_);

    goal_.header.stamp = this->now(); // set current time

    // Goals should only be published here because this is the only place where we
    // apply safety bounds. Exceptions: when killing the drone and when clicking END at init pos traj
    pub_goal_->publish(goal_);
}

void TrajectoryGenerator::stateCB(const snapstack_msgs2::msg::State& msg){
    // RCLCPP_INFO(this->get_logger(), "State is Subscribing %f", msg.pos.z);
    pose_.position.x = msg.pos.x;
    pose_.position.y = msg.pos.y;
    pose_.position.z = msg.pos.z;
    pose_.orientation = msg.quat;
}

void TrajectoryGenerator::resetGoal(){
    // Creating a new goal message should already set this correctly, but just in case
    // Exception: yaw would be 0 instead of current yaw
    goal_.p.x = goal_.p.y = goal_.p.z = 0;
    goal_.v.x = goal_.v.y = goal_.v.z = 0;
    goal_.a.x = goal_.a.y = goal_.a.z = 0;
    goal_.j.x = goal_.j.y = goal_.j.z = 0;
    //goal_.s.x = goal_.s.y = goal_.s.z = 0;
    goal_.psi = quat2yaw(pose_.orientation); goal_.dpsi = 0;
//    goal_.power = false;
    // reset_xy_int and  reset_z_int are not used
    goal_.mode_xy = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
    goal_.mode_z = snapstack_msgs2::msg::Goal::MODE_POSITION_CONTROL;
}

// Utils
snapstack_msgs2::msg::Goal TrajectoryGenerator::simpleInterpolation(const snapstack_msgs2::msg::Goal& current,
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

// for new snapstack messages
snapstack_msgs2::msg::Goal TrajectoryGenerator::simpleInterpolation(const snapstack_msgs2::msg::Goal& current,
        const snapstack_msgs2::msg::Goal& dest_pos, double dest_yaw, double vel, double vel_yaw,
        double dist_thresh, double yaw_thresh, double dt, bool& finished){
    snapstack_msgs2::msg::Goal goal;
    // interpolate from current goal pos to the initial goal pos
    double Dx = dest_pos.p.x - current.p.x;
    double Dy = dest_pos.p.y - current.p.y;
    double dist = sqrt(Dx*Dx + Dy*Dy);

    double delta_yaw = dest_yaw - current.psi;
    delta_yaw = wrap(delta_yaw);

    bool dist_far = dist > dist_thresh;
    bool yaw_far  = fabs(delta_yaw) > yaw_thresh;
    finished = not dist_far and not yaw_far;  // both are close

    bool accel_for_vel = 0.1;

    goal.p.z = dest_pos.p.z;  // this should be alt_ and the altitude where the drone took off too
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
        goal.p.x = dest_pos.p.x;
        goal.p.y = dest_pos.p.y;

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

double TrajectoryGenerator::quat2yaw(const geometry_msgs::msg::Quaternion& q){
    double yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z));
    return yaw;
}

double TrajectoryGenerator::saturate(double val, double low, double high){
    if(val > high)
        val = high;
    else if(val < low)
        val = low;

    return val;
}

double TrajectoryGenerator::wrap(double val){
    if(val > M_PI)
        val -= 2.0*M_PI;
    if(val < -M_PI)
        val += 2.0*M_PI;
    return val;
}

} /* namespace */

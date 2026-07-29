/**
 * @file cbf_safety_filter_node.cpp
 * @brief Acceleration-level CBF (HOCBF) safety filter for spherical obstacle
 *        avoidance. Sits between TrajectoryGenerator and the outer-loop
 *        controller: subscribes to the nominal Goal stream + a live obstacle
 *        pose, republishes a corrected Goal stream with p/v/a kept as exact
 *        derivatives of each other.
 *
 * Wiring (no changes needed to TrajectoryGenerator):
 *   remap trajectory_generator_ros2's "goal" publisher -> "goal_nominal"
 *   this node subscribes "goal_nominal", publishes "goal"
 *
 * Dynamics: double integrator on position, x = [p; v], u = a
 *   x_dot = [0 I; 0 0] x + [0; I] u
 *
 * The filter runs its own persistent double-integrator state (p_filt_,
 * v_filt_), driven by a critically-damped PD tracker of the incoming
 * nominal reference plus its feedforward acceleration:
 *   a_track = a_nom + wn_track^2*(p_nom - p_filt) + 2*wn_track*(v_nom - v_filt)
 * a_nom alone has zero dependence on the filtered state (it's an open-loop
 * feedforward computed from trajectory time), so without this tracking term
 * any HOCBF-induced deviation has no restoring force -- error dynamics are
 * error_ddot = 0 whenever the constraint isn't binding, which never
 * reconverges and forces a hard, discontinuous snap back onto nominal later.
 * With the tracker, p_filt_/v_filt_ continuously and smoothly converge onto
 * (and then track) the nominal trajectory whenever the obstacle isn't a
 * factor, so no discrete active/inactive state machine is needed.
 *
 * Barrier for a sphere at obstacle_center_ with radius obstacle_radius_:
 *   h(p)   = ||p - o||^2 - r^2                (relative degree 2 in u)
 *   psi1   = h_dot + alpha1*h
 *   HOCBF constraint (affine in a): psi1_dot + alpha2*psi1 >= 0
 *     => c'*a + d >= 0,  c = 2*(p-o),
 *        d = 2*||v||^2 + 2*alpha1*(p-o)'*v + alpha2*psi1
 *   Closed-form projection of a_track onto that half-space (single obstacle
 *   active at a time -> no QP solver needed).
 */

#include <rclcpp/rclcpp.hpp>

#include <snapstack_msgs2/msg/goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Core>
#include <string>

using std::placeholders::_1;

class CbfSafetyFilter : public rclcpp::Node
{
public:
    CbfSafetyFilter() : Node("cbf_safety_filter")
    {
        this->declare_parameter<std::string>("goal_in_topic", "goal_nominal");
        this->declare_parameter<std::string>("goal_out_topic", "goal");
        this->declare_parameter<std::string>("obstacle_topic", "/obstacle/mocap/pose");
        this->declare_parameter<double>("obstacle_radius", 0.5);
        this->declare_parameter<double>("obstacle_timeout", 0.5);
        this->declare_parameter<double>("cbf_alpha1", 2.0);
        this->declare_parameter<double>("cbf_alpha2", 2.0);
        this->declare_parameter<double>("wn_track", 6.0);  // tracking-error natural frequency
        this->declare_parameter<double>("input_rate", 100.0);  // must match trajectory_generator's pub_freq

        this->get_parameter("goal_in_topic", goal_in_topic_);
        this->get_parameter("goal_out_topic", goal_out_topic_);
        this->get_parameter("obstacle_topic", obstacle_topic_);
        this->get_parameter("obstacle_radius", obstacle_radius_);
        this->get_parameter("obstacle_timeout", obstacle_timeout_);
        this->get_parameter("cbf_alpha1", cbf_alpha1_);
        this->get_parameter("cbf_alpha2", cbf_alpha2_);
        this->get_parameter("wn_track", wn_track_);

        double input_rate;
        this->get_parameter("input_rate", input_rate);
        dt_ = (input_rate > 0.0) ? 1.0 / input_rate : 0.01;

        rclcpp::QoS qos_profile(10);
        qos_profile
            .durability(rclcpp::DurabilityPolicy::Volatile)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);

        goal_in_sub_ = this->create_subscription<snapstack_msgs2::msg::Goal>(
            goal_in_topic_, qos_profile,
            std::bind(&CbfSafetyFilter::goalInCB, this, _1));

        obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            obstacle_topic_, qos_profile,
            std::bind(&CbfSafetyFilter::obstacleCB, this, _1));

        goal_out_pub_ = this->create_publisher<snapstack_msgs2::msg::Goal>(goal_out_topic_, 10);

        RCLCPP_INFO(this->get_logger(),
            "CbfSafetyFilter started. '%s' -> '%s', obstacle on '%s', r=%.2f, "
            "alpha1=%.2f alpha2=%.2f wn_track=%.2f dt=%.4f",
            goal_in_topic_.c_str(), goal_out_topic_.c_str(), obstacle_topic_.c_str(),
            obstacle_radius_, cbf_alpha1_, cbf_alpha2_, wn_track_, dt_);
    }

private:

    void obstacleCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        obstacle_center_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        has_obstacle_ = true;
        last_obstacle_time_ = this->now();
    }

    // Closed-form projection of a_track onto the HOCBF half-space constraint
    // for a single sphere. Relative degree 2 (u enters only through v_dot),
    // so a plain first-order CBF is not affine in u -- this is the HOCBF form.
    static Eigen::Vector3d cbfAccelProjection(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                                               const Eigen::Vector3d& a_track,
                                               const Eigen::Vector3d& obs, double r,
                                               double alpha1, double alpha2)
    {
        const Eigen::Vector3d dp = p - obs;
        const double h    = dp.squaredNorm() - r * r;
        const double psi1 = 2.0 * dp.dot(v) + alpha1 * h;

        const Eigen::Vector3d c = 2.0 * dp;
        const double d = 2.0 * v.squaredNorm() + 2.0 * alpha1 * dp.dot(v) + alpha2 * psi1;

        const double margin = c.dot(a_track) + d;
        if (margin >= 0.0 || c.squaredNorm() < 1e-9) {
            return a_track;  // already safe (or degenerate: sitting exactly on the center)
        }
        return a_track - c * (margin / c.squaredNorm());
    }

    void goalInCB(const snapstack_msgs2::msg::Goal::SharedPtr msg)
    {
        const Eigen::Vector3d p_nom(msg->p.x, msg->p.y, msg->p.z);
        const Eigen::Vector3d v_nom(msg->v.x, msg->v.y, msg->v.z);
        const Eigen::Vector3d a_nom(msg->a.x, msg->a.y, msg->a.z);

        if (!filt_init_) {
            p_filt_ = p_nom;
            v_filt_ = v_nom;
            filt_init_ = true;
        }

        const Eigen::Vector3d a_track = a_nom
            + wn_track_ * wn_track_ * (p_nom - p_filt_)
            + 2.0 * wn_track_ * (v_nom - v_filt_);

        Eigen::Vector3d a_safe = a_track;

        if (!has_obstacle_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No obstacle pose received yet on '%s' -- filter tracks nominal unconstrained.",
                obstacle_topic_.c_str());
        } else {
            const double age = (this->now() - last_obstacle_time_).seconds();
            if (age > obstacle_timeout_) {
                // Conservative default: freeze the last known obstacle position and keep
                // avoiding it, rather than silently dropping protection on a sensor dropout.
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Obstacle pose stale (%.2f s) -- holding last known position.", age);
            }

            a_safe = cbfAccelProjection(p_filt_, v_filt_, a_track, obstacle_center_,
                                         obstacle_radius_, cbf_alpha1_, cbf_alpha2_);
        }

        v_filt_ += a_safe * dt_;
        p_filt_ += v_filt_ * dt_;

        snapstack_msgs2::msg::Goal out = *msg;  // keep psi/dpsi/power/mode flags/header/j as-is
        out.p.x = p_filt_.x(); out.p.y = p_filt_.y(); out.p.z = p_filt_.z();
        out.v.x = v_filt_.x(); out.v.y = v_filt_.y(); out.v.z = v_filt_.z();
        out.a.x = a_safe.x();  out.a.y = a_safe.y();  out.a.z = a_safe.z();
        goal_out_pub_->publish(out);
    }

    rclcpp::Subscription<snapstack_msgs2::msg::Goal>::SharedPtr goal_in_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<snapstack_msgs2::msg::Goal>::SharedPtr goal_out_pub_;

    std::string goal_in_topic_;
    std::string goal_out_topic_;
    std::string obstacle_topic_;

    double obstacle_radius_{0.5};
    double obstacle_timeout_{0.5};
    double cbf_alpha1_{2.0};
    double cbf_alpha2_{2.0};
    double wn_track_{6.0};
    double dt_{0.01};

    Eigen::Vector3d obstacle_center_{Eigen::Vector3d::Zero()};
    bool has_obstacle_{false};
    rclcpp::Time last_obstacle_time_{0, 0, RCL_ROS_TIME};

    // Filter (double-integrator) state, always advancing, seeded from the
    // first nominal goal received.
    Eigen::Vector3d p_filt_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_filt_{Eigen::Vector3d::Zero()};
    bool filt_init_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CbfSafetyFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

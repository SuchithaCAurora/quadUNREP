/**
 * @file Reciprocating.hpp
 * @brief Reciprocating trajectory class (back-and-forth along a line with 180 deg yaw turns)
 * @author Russell perez
 * @date 2025-06-27
 */

#pragma once

#include "trajectory_generator_ros2/trajectories/Trajectory.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "snapstack_msgs2/msg/goal.hpp"
#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <string>

namespace trajectory_generator {

/*!
 * Class that represents a reciprocating (back-and-forth) trajectory along a line
 */
class Reciprocating : public Trajectory
{
public:
    Reciprocating(double alt, Eigen::Vector3d A, Eigen::Vector3d B,
                  std::vector<double> v_goals, double a1, double a3, double t_traj, double dt);

    virtual ~Reciprocating();

    void generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                      std::unordered_map<int,std::string>& index_msgs,
                      const rclcpp::Clock::SharedPtr& clock) override;

    snapstack_msgs2::msg::Goal createReciprocatingGoal(double x, double y, double v, double accel, double heading) const;

    void generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          int& pub_index,
                          const rclcpp::Clock::SharedPtr& clock) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax,
                                double zmin, double zmax) override;

private:
    double alt_;
    Eigen::Vector3d A_, B_;
    std::vector<double> v_goals_;
    double a1_, a3_;
    double t_traj_;
    double theta_fwd_, theta_rev_;
    rclcpp::Logger logger_ = rclcpp::get_logger("reciprocating_logger");
};

} // namespace trajectory_generator
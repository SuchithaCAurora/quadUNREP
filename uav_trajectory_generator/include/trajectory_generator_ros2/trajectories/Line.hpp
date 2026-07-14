/**
 * @file Line.hpp
 * @brief Line class
 * @author Aleix Paris
 * @date 2020-02-19
 */

#pragma once

#include "trajectory_generator_ros2/trajectories/Trajectory.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "snapstack_msgs2/msg/goal.hpp"

#include <string>
#include <unordered_map>
#include <Eigen/Core>
#include <vector>

namespace trajectory_generator {

/*!
 * Class that represents a linear trajectory from point A_ to point B_, and back
 */
class Line: public Trajectory
{
public:

    Line(double alt, Eigen::Vector3d A, Eigen::Vector3d B,
         std::vector<double> v_goals, double a1, double a3, double dt);

    virtual ~Line();

    void generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                      std::unordered_map<int,std::string>& index_msgs,
                      const rclcpp::Clock::SharedPtr& clock) override;
    snapstack_msgs2::msg::Goal createLineGoal (double last_x, double last_y,
                                             double v, double accel, double theta) const;

    void generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          int& pub_index,
                          const rclcpp::Clock::SharedPtr& clock) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax,
                                double zmin, double zmax) override;

private:

    double get_d2() const;  // get length of constant velocity segment

    double alt_;  // altitude in m

    Eigen::Vector3d A_, B_;
    std::vector<double> v_goals_;  // norm of goal velocities, for now just 1 element
    double a1_, a3_;  // abs val of acceleration and deceleration (a2 = 0, ct vel segment)
    // note that both are > 0 even though a3_ represents a deceleration

    double theta_;  // angle from line AB to x axis, precalculated for efficiency

    rclcpp::Logger logger_ = rclcpp::get_logger("line_logger");
};

} /* namespace */

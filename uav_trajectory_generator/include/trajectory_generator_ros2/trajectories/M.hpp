/**
 * @file M.hpp
 * @brief M trajectory class (traces out a capital letter M)
 * @author Russell Perez
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

class M : public Trajectory
{
public:
    M(double cx, double cy, double length, double width, double alt,
      std::vector<double> v_goals, double t_traj, double orientation, double dt);

    virtual ~M();

    void generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                      std::unordered_map<int,std::string>& index_msgs,
                      const rclcpp::Clock::SharedPtr& clock) override;

    snapstack_msgs2::msg::Goal createMGoal(double x, double y, double v, double accel, double heading) const;

    void generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          int& pub_index,
                          const rclcpp::Clock::SharedPtr& clock) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax,
                                double zmin, double zmax) override;

private:
    double cx_, cy_;
    double length_, width_;
    double alt_;
    double orientation_;
    std::vector<double> v_goals_;
    double t_traj_;
    rclcpp::Logger logger_ = rclcpp::get_logger("m_logger");
};

} // namespace trajectory_generator
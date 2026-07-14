/**
 * @file Rectangle.hpp
 * @brief Rectangle trajectory class (like Square, but with two different side lengths)
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

/*!
 * Class that represents a rectangular trajectory in the XY plane
 */
class Rectangle : public Trajectory
{
public:
    Rectangle(double alt, double side_a, double side_b, double cx, double cy, double orientation,
              std::vector<double> v_goals, double t_traj, double accel, double dt);

    virtual ~Rectangle();

    void generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                      std::unordered_map<int,std::string>& index_msgs,
                      const rclcpp::Clock::SharedPtr& clock) override;

    snapstack_msgs2::msg::Goal createRectangleGoal(double x, double y, double v, double accel, double heading) const;

    void generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          int& pub_index,
                          const rclcpp::Clock::SharedPtr& clock) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax,
                                double zmin, double zmax) override;

private:
    double alt_;
    double side_a_, side_b_;
    double cx_, cy_;
    double orientation_;
    std::vector<double> v_goals_;
    double t_traj_;
    double accel_;
    rclcpp::Logger logger_ = rclcpp::get_logger("rectangle_logger");
};

} // namespace
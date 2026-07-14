/**
 * @file Square.hpp
 * @brief Square class
 * @author Russell Perez
 * @date 2025-06-20
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
 * Class that represents a square trajectory
 */
class Square: public Trajectory
{
public:

    Square(double alt, double side_length, double cx, double cy, double orientation,
           std::vector<double> v_goals, double t_traj, double accel, double dt);

    virtual ~Square();

    void generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                      std::unordered_map<int,std::string>& index_msgs,
                      const rclcpp::Clock::SharedPtr& clock) override;  // generates the square

    snapstack_msgs2::msg::Goal createSquareGoal(double x, double y, double v, double accel, double heading) const;

    void generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          int& pub_index,
                          const rclcpp::Clock::SharedPtr& clock) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax,
                                double zmin, double zmax) override;

private:
    double alt_;         // altitude in m
    double side_length_; // side length of the square in m
    double cx_, cy_;     // center of the square in m
    double orientation_; // orientation of the square (radians)

    std::vector<double> v_goals_;  // norm of goal velocities
    double t_traj_;                // time to follow each trajectory, s
    double accel_;                 // max acceleration allowed

    rclcpp::Logger logger_ = rclcpp::get_logger("square_logger");
};

} /* namespace */

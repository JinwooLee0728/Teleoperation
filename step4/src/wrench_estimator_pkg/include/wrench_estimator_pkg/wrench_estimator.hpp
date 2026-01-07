// wrench_estimator.hpp

#pragma once
#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>

void estimateWrench(
    const pinocchio::Model& model,                // Robot model
    pinocchio::Data& data,                        // Robot data
    const pinocchio::FrameIndex end_effector_id,  // End effector frame ID
    const Eigen::VectorXd& q,                     // Joint positions
    const Eigen::VectorXd& q_dot,                 // Joint velocities
    const Eigen::VectorXd& tau,                   // Command joint torques
    const Eigen::VectorXd& p0,                    // Initial momentum in joint space
    const double& dt_sec,                         // dt in seconds
    Eigen::VectorXd& wrench_hat,                  // Estimated end effector wrench (OUTPUT)
    Eigen::VectorXd& integral_value               // Integral value (OUTPUT) 
);
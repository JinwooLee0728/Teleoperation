// ==================================================
// Wrench Estimation 1
// ==================================================

// Eigen
#include <Eigen/Dense>

// Pinocchio headers
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

// URDF File
#define URDF_FILENAME             "/home/jinwoo/Desktop/teleop_prj/FACTR_materials/FACTR_Hardware/urdf/factr_teleop_franka.urdf"

// Estimator Tuning Gain
#define K_I                       10

// Wrench estimation function (return integral value and wrench_hat as output parameters)
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
    Eigen::VectorXd& integral_value)              // Integral value (OUTPUT) 
{
    using namespace pinocchio;

    Eigen::VectorXd r(model.nv);
    Eigen::VectorXd p(model.nv);
    Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);

    assert(q.size() == model.nq);
    assert(q_dot.size() == model.nv);
    assert(tau.size() == model.nv);
    assert(integral_value.size() == model.nv);
    assert(wrench_hat.size() == 6);

    // Computing M(q), C(q, q_dot), G(q) matrices
    pinocchio::computeAllTerms(model, data, q, q_dot);

    // Computing Jacobians
    pinocchio::computeFrameJacobian(model, data, q, end_effector_id, pinocchio::LOCAL, J);  //LOCAL_WORLD_ALIGNED
    
    // Computing r(t)
    // ------------------------------------------------------------
    // | r(t) = KI*(p(t)-integral_value-p(0)), p(t) = M(q)*q_dot  |
    // ------------------------------------------------------------
    p = data.M*q_dot;
    r = K_I*(p-integral_value-p0);

    // Updating integral_value
    integral_value.noalias() += (tau+data.C.transpose()*q_dot-data.g+r)*dt_sec;

    // Computing wrench_hat
    // ------------------------------------------------------------
    // | F_K(t) = pinv(J^T)*r(t)                                  |
    // ------------------------------------------------------------
    wrench_hat = J.transpose().completeOrthogonalDecomposition().solve(r);
    
}
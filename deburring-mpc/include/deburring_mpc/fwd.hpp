#ifndef DEBURRING_FWD
#define DEBURRING_FWD

#include <pinocchio/fwd.hpp>
// include pinocchio first
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <pinocchio/spatial/fwd.hpp>

namespace deburring {
// Eigen
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using VectorXd = Eigen::VectorXd;
using Matrix3d = Eigen::Matrix3d;
using MatrixXd = Eigen::MatrixXd;
using ConstVectorRef = Eigen::Ref<const Eigen::VectorXd>;

// Pinocchio
using SE3 = pinocchio::SE3;

// Crocoddyl
using DDP = boost::shared_ptr<crocoddyl::SolverFDDP>;
using IntegratedActionModel =
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler>;
using IntegratedActionData =
    boost::shared_ptr<crocoddyl::IntegratedActionDataEuler>;
using ActionModel = boost::shared_ptr<crocoddyl::ActionModelAbstract>;
using ActionData = boost::shared_ptr<crocoddyl::ActionDataAbstract>;
using DifferentialActionModel =
    boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics>;
using CostModelSum = boost::shared_ptr<crocoddyl::CostModelSum>;
using Contact = boost::shared_ptr<crocoddyl::ContactModelMultiple>;
}  // namespace deburring

#endif  // DEBURRING_FWD
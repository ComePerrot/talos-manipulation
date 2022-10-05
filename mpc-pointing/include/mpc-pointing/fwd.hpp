#ifndef MPC_P_FWD
#define MPC_P_FWD

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <sobec/walk-with-traj/designer.hpp>
#include <sobec/walk-with-traj/model_factory.hpp>

namespace mpc_p {
// Eigen
using Vector3d = Eigen::Vector3d;
using VectorXd = Eigen::VectorXd;
using Matrix3d = Eigen::Matrix3d;
using MatrixXd = Eigen::MatrixXd;

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

// Sobec
using RobotWrapper = sobec::RobotDesigner;
using ModelMaker = sobec::ModelMaker;
using ModelMakerSettings = sobec::ModelMakerSettings;
}  // namespace mpc_p

#endif  // MPC_P_FWD
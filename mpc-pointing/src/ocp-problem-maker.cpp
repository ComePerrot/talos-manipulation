#include "mpc-pointing/ocp.hpp"

namespace mpc_p {
void OCP_Point::buildSolver(const Eigen::VectorXd x0, pinocchio::SE3 oMtarget,
                            const ModelMakerSettings modelMakerSettings) {
  modelMaker_ = ModelMaker(modelMakerSettings, designer_);

  auto runningModels = std::vector<ActionModel>(settings_.horizon_length);

  for (size_t i = 0; i < settings_.horizon_length; i++) {
    runningModels[i] = modelMaker_.formulatePointingTask();
  }

  auto terminalModel = modelMaker_.formulatePointingTask();

  boost::shared_ptr<ShootingProblem> shooting_problem =
      boost::make_shared<ShootingProblem>(x0, runningModels, terminalModel);
  solver_ = boost::make_shared<SolverFDDP>(shooting_problem);

  // Change Torque Reference
  setBalancingTorques();

  // Change Target placmenet
  updateGoalPosition(oMtarget.translation());
  updateGoalRotation(oMtarget.rotation());
}

void OCP_Point::solveFirst(const Eigen::VectorXd x) {
  // horizon settings
  std::vector<Eigen::VectorXd> xs_init;
  std::vector<Eigen::VectorXd> us_init;

  for (std::size_t i = 0; i < settings_.horizon_length; i++) {
    xs_init.push_back(x);
    // Gravity compensation torques
    us_init.push_back(
        boost::static_pointer_cast<crocoddyl::ResidualModelControl>(
            costs(i)->get_costs().at("actuationTask")->cost->get_residual())
            ->get_reference());
  }
  xs_init.push_back(x);

  solver_->solve(xs_init, us_init, 500, false);
}
}  // namespace mpc_p
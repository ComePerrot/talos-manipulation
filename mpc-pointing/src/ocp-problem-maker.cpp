#include "mpc-pointing/ocp.hpp"

namespace mpc_p {
void OCP_Point::buildSolver(const VectorXd x0, SE3 oMtarget,
                            const ModelMakerSettings &modelMakerSettings) {
  modelMaker_ = ModelMaker(modelMakerSettings, designer_);

  auto runningModels = std::vector<ActionModel>(settings_.horizon_length);

  for (size_t i = 0; i < settings_.horizon_length; i++) {
    runningModels[i] = modelMaker_.formulateRunningPointingTask();
  }

  // Terminal model
  auto terminalModel = modelMaker_.formulateTerminalPointingTask();

  boost::shared_ptr<ShootingProblem> shooting_problem =
      boost::make_shared<ShootingProblem>(x0, runningModels, terminalModel);
  solver_ = boost::make_shared<SolverFDDP>(shooting_problem);

  // Change Torque Reference
  setBalancingTorques();

  // Change Target placemenet
  updateGoalPosition(oMtarget.translation());
  updateGoalRotation(oMtarget.rotation());

  // Deactivate Target cost at the beginning
  for (size_t modelIndex = 0; modelIndex <= settings_.horizon_length;
       modelIndex++) {
    changeGoalCostActivation(modelIndex, false);
  }
}

void OCP_Point::solveFirst(const VectorXd x) {
  // horizon settings
  std::vector<VectorXd> xs_init;
  std::vector<VectorXd> us_init;

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
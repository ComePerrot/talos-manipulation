#include "deburring_mpc/ocp.hpp"

namespace deburring {
// Functions to interact with ddp
void OCP::setWarmStart(const std::vector<VectorXd> &warm_xs,
                       const std::vector<VectorXd> &warm_us) {
  solver_->set_xs(warm_xs);
  solver_->set_us(warm_us);
}
void OCP::recede() {
  solver_->get_problem()->circularAppend(
      solver_->get_problem()->get_runningModels()[0],
      solver_->get_problem()->get_runningDatas()[0]);
}
void OCP::changeTarget(const size_t index,
                       const Eigen::Ref<const Vector3d> &position) {
  boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(
      costs(index)->get_costs().at("gripperPosition")->cost->get_residual())
      ->set_reference(position);
}
void OCP::setBalancingTorques() {
  for (size_t node_index = 0; node_index < settings_.horizon_length;
       node_index++) {
    VectorXd x_ref = boost::static_pointer_cast<crocoddyl::ResidualModelState>(
                         costs(node_index)
                             ->get_costs()
                             .at("postureTask")
                             ->cost->get_residual())
                         ->get_reference();

    VectorXd balancing_torques;
    balancing_torques.resize((long)iam(node_index)->get_nu());
    iam(node_index)->quasiStatic(ada(node_index), balancing_torques, x_ref);

    boost::static_pointer_cast<crocoddyl::ResidualModelControl>(
        costs(node_index)
            ->get_costs()
            .at("actuationTask")
            ->cost->get_residual())
        ->set_reference(balancing_torques);
  }
}
void OCP::updateGoalPosition(const Eigen::Ref<const Vector3d> &position) {
  for (size_t node_index = 0; node_index <= settings_.horizon_length;
       node_index++) {
    boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(
        costs(node_index)
            ->get_costs()
            .at("gripperPosition")
            ->cost->get_residual())
        ->set_reference(position);
  }
}
void OCP::updateGoalRotation(const Eigen::Ref<const Matrix3d> &rotation) {
  for (size_t node_index = 0; node_index <= settings_.horizon_length;
       node_index++) {
    boost::static_pointer_cast<crocoddyl::ResidualModelFrameRotation>(
        costs(node_index)
            ->get_costs()
            .at("gripperRotation")
            ->cost->get_residual())
        ->set_reference(rotation);
  }
}
void OCP::changeGoalCostActivation(const size_t index, const bool value) {
  costs(index)->get_costs().at("gripperPosition")->active = value;
  costs(index)->get_costs().at("gripperRotation")->active = value;
}
void OCP::changeGoalTrackingWeights(double weight) {
  for (size_t node_index = 0; node_index <= settings_.horizon_length;
       node_index++) {
    costs(node_index)->get_costs().at("gripperPosition")->weight = weight;
  }
}
void OCP::changePostureReference(const size_t index,
                                 const Eigen::Ref<const VectorXd> reference) {
  boost::static_pointer_cast<crocoddyl::ResidualModelState>(
      costs(index)->get_costs().at("postureTask")->cost->get_residual())
      ->set_reference(reference);
}

const VectorXd &OCP::getFinalPosture() { return (solver_->get_xs().back()); }

ActionModel OCP::ama(const unsigned long time) {
  if (time == settings_.horizon_length) {
    return solver_->get_problem()->get_terminalModel();
  } else {
    return solver_->get_problem()->get_runningModels()[time];
  }
}

IntegratedActionModel OCP::iam(const unsigned long time) {
  return boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
      ama(time));
}

DifferentialActionModel OCP::dam(const unsigned long time) {
  return boost::static_pointer_cast<
      crocoddyl::DifferentialActionModelContactFwdDynamics>(
      iam(time)->get_differential());
}

CostModelSum OCP::costs(const unsigned long time) {
  return dam(time)->get_costs();
}

ActionData OCP::ada(const unsigned long time) {
  return solver_->get_problem()->get_runningDatas()[time];
}
}  // namespace deburring
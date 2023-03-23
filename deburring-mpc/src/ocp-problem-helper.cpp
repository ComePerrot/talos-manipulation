#include "deburring_mpc/ocp.hpp"

namespace deburring {
// Functions to interact with ddp
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
  for (size_t modelIndex = 0; modelIndex < settings_.horizon_length;
       modelIndex++) {
    VectorXd x_ref = boost::static_pointer_cast<crocoddyl::ResidualModelState>(
                         costs(modelIndex)
                             ->get_costs()
                             .at("postureTask")
                             ->cost->get_residual())
                         ->get_reference();

    VectorXd balancingTorque;
    balancingTorque.resize((long)iam(modelIndex)->get_nu());
    iam(modelIndex)->quasiStatic(ada(modelIndex), balancingTorque, x_ref);

    boost::static_pointer_cast<crocoddyl::ResidualModelControl>(
        costs(modelIndex)
            ->get_costs()
            .at("actuationTask")
            ->cost->get_residual())
        ->set_reference(balancingTorque);
  }
}
void OCP::updateGoalPosition(const Eigen::Ref<const Vector3d> &position) {
  for (size_t modelIndex = 0; modelIndex <= settings_.horizon_length;
       modelIndex++) {
    boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(
        costs(modelIndex)
            ->get_costs()
            .at("gripperPosition")
            ->cost->get_residual())
        ->set_reference(position);
  }
}
void OCP::updateGoalRotation(const Eigen::Ref<const Matrix3d> &rotation) {
  for (size_t modelIndex = 0; modelIndex <= settings_.horizon_length;
       modelIndex++) {
    boost::static_pointer_cast<crocoddyl::ResidualModelFrameRotation>(
        costs(modelIndex)
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
void OCP::changeGoaleTrackingWeights(double weight) {
  for (size_t modelIndex = 0; modelIndex < settings_.horizon_length;
       modelIndex++) {
    costs(modelIndex)->get_costs().at("gripperPosition")->weight = weight;
  }
}
void OCP::changePostureReference(
    const size_t index, const Eigen::Ref<const VectorXd> reference) {
  boost::static_pointer_cast<crocoddyl::ResidualModelState>(
      costs(index)->get_costs().at("postureTask")->cost->get_residual())
      ->set_reference(reference);
}

const VectorXd& OCP::getFinalPosture(){
  return (solver_->get_xs().back());
}

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
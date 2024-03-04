#include "deburring_mpc/ocp.hpp"

namespace deburring {
void OCP::buildSolver(const VectorXd x0, SE3 oMtarget) {
  designer_.updateReducedModel(x0);

  state_ = boost::make_shared<crocoddyl::StateMultibody>(
      boost::make_shared<pinocchio::Model>(designer_.get_rmodel()));
  actuation_ =
      boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);

  auto running_models = std::vector<ActionModel>(settings_.horizon_length);

  for (size_t node_index = 0; node_index < settings_.horizon_length;
       node_index++) {
    running_models[node_index] = formulatePointingTask();
  }

  // Terminal model
  auto terminal_model = formulateTerminalPointingTask();

  boost::shared_ptr<crocoddyl::ShootingProblem> shooting_problem =
      boost::make_shared<crocoddyl::ShootingProblem>(x0, running_models,
                                                     terminal_model);
  solver_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);

  reset(x0, oMtarget);
}

void OCP::reset(const ConstVectorRef &x0, const SE3 &oMtarget) {
  // Change References
  VectorXd posture_reference = x0;
  posture_reference.tail(designer_.get_rmodel().nv) =
      VectorXd::Zero(designer_.get_rmodel().nv);
  for (size_t node_index = 0; node_index <= settings_.horizon_length;
       node_index++) {
    changePostureReference(node_index, posture_reference);
  }
  setBalancingTorques();

  // Change Target placemenet
  updateGoalPosition(oMtarget.translation());
  updateGoalRotation(oMtarget.rotation());

  // Deactivate Target cost at the beginning
  for (size_t node_index = 0; node_index <= settings_.horizon_length;
       node_index++) {
    changeGoalCostActivation(node_index, false);
  }
}

void OCP::solveFirst(const VectorXd x) {
  // horizon settings
  std::vector<VectorXd> xs_init;
  std::vector<VectorXd> us_init;

  for (std::size_t i = 0; i < settings_.horizon_length; i++) {
    xs_init.push_back(x);
    // Gravity compensation torques
    us_init.push_back(
        boost::static_pointer_cast<crocoddyl::ResidualModelControl>(
            costs(i)
                ->get_costs()
                .at("controlRegularization")
                ->cost->get_residual())
            ->get_reference());
  }
  xs_init.push_back(x);

  solver_->solve(xs_init, us_init, 500, false);
}

ActionModel OCP::formulatePointingTask() {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  CostModelSum costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts);
  defineFeetWrenchCost(costs, 0);

  // Safety constraints
  defineStateLimits(costs, settings_.w_state_limits, settings_.limit_scale,
                    settings_.limit_speed);
  defineControlLimits(costs, settings_.w_control_limit, settings_.limit_scale);

  // Equilibrium constraints
  defineCoMPosition(costs, settings_.w_com_pos);

  // Regulation task
  defineStateRegularization(costs, settings_.w_state_reg);
  defineControlRegularization(costs, settings_.w_control_reg);

  // End effector task
  defineGripperPlacement(costs, settings_.w_gripper_pos,
                         settings_.w_gripper_rot);
  defineGripperVelocity(costs, settings_.w_gripper_vel);

  DifferentialActionModel running_dam =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  setWristArmature(running_dam);

  ActionModel running_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
          running_dam, settings_.time_step);

  return running_model;
}

ActionModel OCP::formulateTerminalPointingTask() {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  CostModelSum costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts);
  defineFeetWrenchCost(costs, 0);

  // Safety constraints
  defineStateLimits(costs, settings_.w_state_limits, settings_.limit_scale,
                    settings_.limit_speed);

  // Equilibrium constraints
  defineCoMPosition(costs, settings_.w_com_pos);

  // Regulation task
  defineStateRegularization(costs, settings_.w_state_reg);

  // End effector task
  defineGripperPlacement(costs, settings_.w_gripper_pos,
                         settings_.w_gripper_rot);
  defineGripperVelocity(costs, settings_.w_gripper_vel);

  DifferentialActionModel terminal_dam =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  setWristArmature(terminal_dam);

  ActionModel terminal_model =
      boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminal_dam,
                                                                0);

  return terminal_model;
}

void OCP::setWristArmature(DifferentialActionModel DAM) {
  auto pin_model_ = designer_.get_rmodel();
  VectorXd armature = Eigen::VectorXd::Zero(pin_model_.nv);

  const std::vector<std::string> joint_names = {
      "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"};
  const VectorXd armature_values =
      (Eigen::Matrix<double, 3, 1, Eigen::DontAlign>() << 0.1, 0.1, 0.1)
          .finished();  // Values corresponding to each joint name

  for (size_t i = 0; i < joint_names.size(); ++i) {
    if (pin_model_.existJointName(joint_names[i])) {
      armature[static_cast<long>(pin_model_.getJointId(joint_names[i])) + 4] =
          armature_values[static_cast<Eigen::Index>(i)];
    }
  }

  DAM->set_armature(armature);
}

}  // namespace deburring
#include "deburring_mpc/ocp.hpp"

namespace deburring {
void OCP::defineFeetContact(Contact &contactCollector) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelLeft =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_lf_id(), designer_.get_lf_frame(),
          actuation_->get_nu(), Eigen::Vector2d(0., 4.));

  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelRight =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_rf_id(), designer_.get_rf_frame(),
          actuation_->get_nu(), Vector2d(0., 4.));

  contactCollector->addContact(designer_.get_lf_name(), ContactModelLeft,
                               false);
  contactCollector->addContact(designer_.get_rf_name(), ContactModelRight,
                               false);

  contactCollector->changeContactStatus(designer_.get_lf_name(), true);
  contactCollector->changeContactStatus(designer_.get_rf_name(), true);
}

void OCP::definePostureTask(CostModelSum &costCollector,
                                  const double wStateReg) {
  if (settings_.stateWeights.size() != designer_.get_rmodel().nv * 2) {
    throw std::invalid_argument("State weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.stateWeights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> postureModel =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationWQ,
          boost::make_shared<crocoddyl::ResidualModelState>(
              state_, designer_.get_x0(), actuation_->get_nu()));

  costCollector.get()->addCost("postureTask", postureModel, wStateReg, true);
}

void OCP::defineActuationTask(CostModelSum &costCollector,
                                    const double wControlReg) {
  if (settings_.controlWeights.size() != (int)actuation_->get_nu()) {
    throw std::invalid_argument("Control weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.controlWeights);  //.tail(actuation->get_nu())

  boost::shared_ptr<crocoddyl::CostModelAbstract> actuationModel =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationWQ,
          boost::make_shared<crocoddyl::ResidualModelControl>(
              state_, actuation_->get_nu()));
  costCollector.get()->addCost("actuationTask", actuationModel, wControlReg,
                               true);
}

void OCP::defineJointLimits(CostModelSum &costCollector,
                                  const double wLimit,
                                  const double boundScale) {
  Eigen::VectorXd lower_bound(2 * state_->get_nv()),
      upper_bound(2 * state_->get_nv());
  double inf = 9999.0;
  lower_bound << Eigen::VectorXd::Constant(6, -inf),
      designer_.get_rmodel().lowerPositionLimit.tail(
          static_cast<Eigen::Index>(state_->get_nq() - 7)),
      Eigen::VectorXd::Constant(static_cast<Eigen::Index>(state_->get_nv()),
                                -inf);

  upper_bound << Eigen::VectorXd::Constant(6, inf),
      designer_.get_rmodel().upperPositionLimit.tail(
          static_cast<Eigen::Index>(state_->get_nq() - 7)),
      Eigen::VectorXd::Constant(static_cast<Eigen::Index>(state_->get_nv()),
                                inf);

  crocoddyl::ActivationBounds bounds =
      crocoddyl::ActivationBounds(lower_bound, upper_bound, boundScale);

  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationQB =
      boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);
  boost::shared_ptr<crocoddyl::CostModelAbstract> jointLimitCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationQB,
          boost::make_shared<crocoddyl::ResidualModelState>(
              state_, actuation_->get_nu()));

  costCollector.get()->addCost("jointLimits", jointLimitCost, wLimit, true);
}

void OCP::defineCoMPosition(CostModelSum &costCollector,
                                  const double wPCoM) {
  Vector3d refPosition = designer_.get_com_position();
  boost::shared_ptr<crocoddyl::CostModelAbstract> CoMPositionCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelCoMPosition>(
                      state_, refPosition, actuation_->get_nu()));

  costCollector.get()->addCost("comPosition", CoMPositionCost, wPCoM, true);
}

void OCP::defineGripperPlacement(CostModelSum &costCollector,
                                       const double wGripperPos,
                                       const double wGripperRot) {
  pinocchio::SE3 goalPlacement = pinocchio::SE3::Identity();

  // Position
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripperPositionCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_,
          boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(3, 0.02),
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_end_effector_id(), goalPlacement.translation(),
              actuation_->get_nu()));

  costCollector.get()->addCost("gripperPosition", gripperPositionCost,
                               wGripperPos, true);

  // Orientation
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripperRotationCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
                      state_, designer_.get_end_effector_id(),
                      goalPlacement.rotation(), actuation_->get_nu()));

  costCollector.get()->addCost("gripperRotation", gripperRotationCost,
                               wGripperRot, true);
}

void OCP::defineGripperVelocity(CostModelSum &costCollector,
                                      const double wGripperVel) {
  pinocchio::Motion goalMotion = pinocchio::Motion(Eigen::VectorXd::Zero(6));
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripperVelocityCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(
                      state_, designer_.get_end_effector_id(), goalMotion,
                      pinocchio::WORLD, actuation_->get_nu()));

  costCollector.get()->addCost("gripperVelocity", gripperVelocityCost,
                               wGripperVel, true);
}

}  // namespace deburring
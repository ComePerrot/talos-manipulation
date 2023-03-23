#include "deburring_mpc/robot_designer.hpp"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace deburring {

RobotDesigner::RobotDesigner() {}

RobotDesigner::RobotDesigner(const RobotDesignerSettings &settings) {
  initialize(settings);
}

void RobotDesigner::initialize(const RobotDesignerSettings &settings) {
  settings_ = settings;

  // COMPLETE MODEL //
  if (settings_.robot_description.size() > 0) {
    pinocchio::urdf::buildModelFromXML(settings_.robot_description,
                                       pinocchio::JointModelFreeFlyer(),
                                       rmodel_complete_);
    std::cout << "### Build pinocchio model from rosparam robot_description."
              << std::endl;
  } else if (settings_.urdf_path.size() > 0) {
    pinocchio::urdf::buildModel(
        settings_.urdf_path, pinocchio::JointModelFreeFlyer(), rmodel_complete_);
    std::cout << "### Build pinocchio model from urdf file." << std::endl;
  } else {
    throw std::invalid_argument(
        "the urdf file, or robot_description must be specified.");
  }
  rdata_complete_ = pinocchio::Data(rmodel_complete_);

  pinocchio::srdf::loadReferenceConfigurations(rmodel_complete_,
                                               settings_.srdf_path, false);
  pinocchio::srdf::loadRotorParameters(rmodel_complete_, settings_.srdf_path,
                                       false);
  q0_complete_ = rmodel_complete_.referenceConfigurations["half_sitting"];
  v0_complete_ = VectorXd::Zero(rmodel_complete_.nv);

  // REDUCED MODEL //

  if (settings_.controlled_joints_names[0] != "root_joint") {
    throw std::invalid_argument(
        "the joint at index 0 must be called 'root_joint' ");
  }

  // Check if listed joints belong to model
  for (std::vector<std::string>::const_iterator it =
           settings_.controlled_joints_names.begin();
       it != settings_.controlled_joints_names.end(); ++it) {
    const std::string &joint_name = *it;
    std::cout << joint_name << std::endl;
    std::cout << rmodel_complete_.getJointId(joint_name) << std::endl;
    if (not(rmodel_complete_.existJointName(joint_name))) {
      std::cout << "joint: " << joint_name << " does not belong to the model"
                << std::endl;
    }
  }

  // making list of blocked joints
  std::vector<unsigned long> locked_joints_id;
  for (std::vector<std::string>::const_iterator it =
           rmodel_complete_.names.begin() + 1;
       it != rmodel_complete_.names.end(); ++it) {
    const std::string &joint_name = *it;
    if (std::find(settings_.controlled_joints_names.begin(),
                  settings_.controlled_joints_names.end(),
                  joint_name) == settings_.controlled_joints_names.end()) {
      locked_joints_id.push_back(rmodel_complete_.getJointId(joint_name));
    }
  }

  rmodel_ = pinocchio::buildReducedModel(rmodel_complete_, locked_joints_id,
                                         q0_complete_);
  rdata_ = pinocchio::Data(rmodel_);

  pinocchio::srdf::loadReferenceConfigurations(rmodel_, settings_.srdf_path,
                                               false);
  pinocchio::srdf::loadRotorParameters(rmodel_, settings_.srdf_path, false);
  q0_ = rmodel_.referenceConfigurations["half_sitting"];
  v0_ = Eigen::VectorXd::Zero(rmodel_.nv);
  x0_.resize(rmodel_.nq + rmodel_.nv);
  x0_ << q0_, v0_;
  // Generating list of indices for controlled joints //
  for (std::vector<std::string>::const_iterator it = rmodel_.names.begin() + 1;
       it != rmodel_.names.end(); ++it) {
    const std::string &joint_name = *it;
    if (std::find(settings_.controlled_joints_names.begin(),
                  settings_.controlled_joints_names.end(),
                  joint_name) != settings_.controlled_joints_names.end()) {
      controlled_joints_ids_.push_back(rmodel_complete_.getJointId(joint_name));
    }
  }

  left_foot_id_ = rmodel_.getFrameId(settings_.left_foot_name);
  right_foot_id_ = rmodel_.getFrameId(settings_.right_foot_name);

  updateReducedModel(q0_);
  is_initialized_ = true;
}

void RobotDesigner::updateReducedModel(const ConstVectorRef &x) {
  /** x is the reduced posture, or contains the reduced posture in the first
   * elements */
  pinocchio::forwardKinematics(rmodel_, rdata_, x.head(rmodel_.nq));
  pinocchio::updateFramePlacements(rmodel_, rdata_);
  com_position_ =
      pinocchio::centerOfMass(rmodel_, rdata_, x.head(rmodel_.nq), false);
  lf_position_ = rdata_.oMf[left_foot_id_].translation();
  rf_position_ = rdata_.oMf[right_foot_id_].translation();
}

void RobotDesigner::updateCompleteModel(const ConstVectorRef &x) {
  /** x is the complete posture, or contains the complete posture in the first
   * elements */
  pinocchio::forwardKinematics(rmodel_complete_, rdata_complete_,
                               x.head(rmodel_complete_.nq));
  pinocchio::updateFramePlacements(rmodel_complete_, rdata_complete_);
  com_position_ = pinocchio::centerOfMass(rmodel_complete_, rdata_complete_,
                                          x.head(rmodel_complete_.nq), false);
  lf_position_ = rdata_.oMf[left_foot_id_].translation();
  rf_position_ = rdata_.oMf[right_foot_id_].translation();
}

void RobotDesigner::updateModelLimits(
    const Eigen::VectorXd lower_position_limit,
    const Eigen::VectorXd upper_Position_limit) {
  if ((rmodel_.lowerPositionLimit.size() != lower_position_limit.size()) ||
      (rmodel_.upperPositionLimit.size() != upper_Position_limit.size())) {
    throw std::runtime_error("Provided limit vector size does not match");
  }
  rmodel_.lowerPositionLimit = lower_position_limit;
  rmodel_.upperPositionLimit = upper_Position_limit;
}

void RobotDesigner::addEndEffectorFrame(std::string end_effector_name,
                                        std::string parent_name,
                                        pinocchio::SE3 end_effector_placement) {
  pinocchio::FrameIndex parent_id = rmodel_.getFrameId(parent_name);
  pinocchio::Frame parent_frame = rmodel_.frames[parent_id];

  rmodel_.addBodyFrame(end_effector_name, parent_frame.parent,
                       parent_frame.placement.act(end_effector_placement),
                       (int)parent_id);

  end_effector_id_ = rmodel_.getFrameId(end_effector_name);
  rdata_ = pinocchio::Data(rmodel_);
}

const SE3 &RobotDesigner::get_lf_frame() {
  return rdata_.oMf[left_foot_id_];
}

const SE3 &RobotDesigner::get_rf_frame() {
  return rdata_.oMf[right_foot_id_];
}

const SE3 &RobotDesigner::get_end_effector_frame() {
  return rdata_.oMf[end_effector_id_];
}

double RobotDesigner::get_robot_mass() {
  robot_mass_ = 0;
  for (pinocchio::Inertia &I : rmodel_.inertias) robot_mass_ += I.mass();
  return robot_mass_;
}

}  // namespace deburring

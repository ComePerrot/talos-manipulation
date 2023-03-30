#include "deburring_mpc/mpc.hpp"

namespace deburring {

#define PI 3.14159265

MPC::MPC(const MPCSettings &mpc_settings, const OCPSettings &ocp_settings,
         const RobotDesigner &designer)
    : settings_(mpc_settings),
      designer_(designer),
      OCP_(ocp_settings, designer_) {
  backward_offset_.translation().z() = settings_.backward_offset;
  goal_weight_ = OCP_.get_settings().w_gripper_pos;
  for (auto offset : settings_.holes_offsets) {
    holes_offsets_.push_back(SE3(Matrix3d::Identity(), offset));
  }
}

void MPC::initialize(const ConstVectorRef &q0, const ConstVectorRef &v0,
                     const SE3 &toolMtarget) {
  controlled_joints_ids_ = designer_.get_controlled_joints_ids();
  x_internal_.resize(designer_.get_rmodel().nq + designer_.get_rmodel().nv);

  x0_.resize(designer_.get_rmodel().nq + designer_.get_rmodel().nv);
  x0_ << shapeState(q0, v0);
  designer_.updateReducedModel(x0_);
  // designer_.updateCompleteModel(q0);

  // Setup target
  setTarget(toolMtarget);

  // Init OCP
  OCP_.initialize(x0_, oMtarget_);
  u0_ = OCP_.get_torque();
  K0_ = OCP_.get_gain();

  initialized_ = true;
}

void MPC::iterate(const ConstVectorRef &q_current,
                  const ConstVectorRef &v_current, const SE3 &toolMtarget) {
  iterate(shapeState(q_current, v_current), toolMtarget);
}

void MPC::iterate(const VectorXd &x0, const SE3 &toolMtarget) {
  x0_ = x0;

  designer_.updateReducedModel(x0_);

  updateTarget(toolMtarget);
  updateOCP();

  OCP_.solve(x0_);

  u0_ = OCP_.get_torque();
  K0_ = OCP_.get_gain();
}

void MPC::setTarget(const SE3 &toolMtarget) {
  // Setup target
  number_holes_ = settings_.holes_offsets.size();

  //  Define oMtarget
  if (settings_.use_mocap == 1 || settings_.use_mocap == 2) {
    toolMhole_ = toolMtarget.act(holes_offsets_[current_hole_]);
    oMtarget_ = designer_.get_end_effector_frame().act(toolMhole_);
  } else {
    oMtarget_.translation() = settings_.target_position;

    // To have the gripper_left_fingertip_3_link in the right orientation we
    // need :
    //   90° around the y axis
    //   180° around the z axis
    double beta = -PI * 0.5;
    double gamma = PI;

    Matrix3d rotationY;
    rotationY.row(0) << cos(beta), 0, -sin(beta);
    rotationY.row(1) << 0, 1, 0;
    rotationY.row(2) << sin(beta), 0, cos(beta);
    Matrix3d rotationZ;
    rotationZ.row(0) << cos(gamma), sin(gamma), 0;
    rotationZ.row(1) << -sin(gamma), cos(gamma), 0;
    rotationZ.row(2) << 0, 0, 1;

    oMtarget_.rotation() = rotationZ * rotationY;
  }

  //  Setup list_oMholes
  setHolesPlacement();
}

void MPC::setHolesPlacement() {
  if (list_oMhole_.empty()) {
    for (size_t h = 0; h < number_holes_; h++) {
      list_oMhole_.push_back(oMtarget_.act(holes_offsets_[h]));
    }
  } else {
    for (size_t h = 0; h < number_holes_; h++) {
      list_oMhole_[h] = oMtarget_.act(holes_offsets_[h]);
    }
  }
}

void MPC::updateTarget(const SE3 &toolMtarget) {
  if (settings_.use_mocap == 0 || settings_.use_mocap == 1) {
    toolMhole_ =
        designer_.get_end_effector_frame().actInv(list_oMhole_[current_hole_]);
    position_error_ = toolMhole_.translation().norm();
  } else if (settings_.use_mocap == 2) {
    toolMhole_ = toolMtarget.act(holes_offsets_[current_hole_]);

    position_error_ = toolMhole_.translation().norm();

    if (position_error_ < 0.05 && drilling_state_ == 2) {
      oMtarget_ = designer_.get_end_effector_frame().act(toolMhole_);

      setHolesPlacement();
      OCP_.updateGoalPosition(list_oMhole_[current_hole_].translation());
    }
  }
}

void MPC::updateOCP() {
  switch (drilling_state_) {
    case 0:  // Initialization
      if (iteration_ < settings_.T_initialization) {
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 1:  // Approaching target
      if (iteration_ == 0) {
        std::cout << "Initiating movement" << std::endl;
      }
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeGoalCostActivation(OCP_.get_horizon_length() - iteration_,
                                      true);
        iteration_++;
      } else {
        std::cout << "Approach phase over" << std::endl;

        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 2:  // Waiting for arm to stabilize (and closing loop on MOCAP
             // feedback)
      if (iteration_ == 0) {
        std::cout << "Waiting for arm to stabilize" << std::endl;
      }
      if (iteration_ < settings_.T_stabilization) {
        iteration_++;
      } else {
        if (position_error_ > 0.05) {
          iteration_ = 0;
          drilling_state_ = 6;
        } else {
          iteration_ = 0;
          drilling_state_++;
        }
      }
      break;

    case 3:  // Increasing gain
      if (iteration_ == 0) {
        std::cout << "Initiating increase gain phase" << std::endl;
      }
      if (settings_.use_gain_scheduling == 1 &&
          position_error_ > settings_.precision_threshold &&
          goal_weight_ < settings_.gain_schedulig_max_weight) {
        goal_weight_ += settings_.gain_schedulig_slope;

        OCP_.changeGoalTrackingWeights(goal_weight_);

        iteration_++;
      } else {
        std::cout << "Gain Scheduling over" << std::endl;

        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 4:  // Drilling
      if (iteration_ == 0) {
        std::cout << "Initiating drilling phase" << std::endl;
      }
      if (iteration_ <= settings_.T_drilling) {
        iteration_++;
      } else {
        std::cout << "Drilling over" << std::endl;

        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 5:  // Getting out of hole
      if (iteration_ == 0) {
        std::cout << "Getting out of the hole" << std::endl;
        if (settings_.use_gain_scheduling == 1) {
          goal_weight_ = OCP_.get_settings().w_gripper_pos;
          OCP_.changeGoalTrackingWeights(goal_weight_);
        }
        oMdisengaged_target_ =
            list_oMhole_[current_hole_].act(backward_offset_);
      }
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeTarget(OCP_.get_horizon_length() - iteration_,
                          oMdisengaged_target_.translation());
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_++;
      }
      break;

    case 6:  // Switching to next target
      if (iteration_ == 0) {
        if (current_hole_ < (number_holes_ - 1)) {
          current_hole_++;
          std::cout << "Switching to target: " << current_hole_ + 1
                    << std::endl;
          iteration_++;
        } else {
          std::cout << "Going back to inital position" << std::endl;
          drilling_state_ = 7;
        }
      }
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeTarget(OCP_.get_horizon_length() - iteration_,
                          list_oMhole_[current_hole_].translation());
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = 2;
      }
      break;

    case 7:  // Returning to initial position
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeGoalCostActivation(OCP_.get_horizon_length() - iteration_,
                                      false);
        iteration_++;
      } else {
        std::cout << "Robot is back to initial position" << std::endl;

        iteration_ = 0;
        drilling_state_++;
      }
      break;
  }
}

const VectorXd &MPC::shapeState(const ConstVectorRef &q,
                                const ConstVectorRef &v) {
  if (q.size() == designer_.get_rmodel_complete().nq &&
      v.size() == designer_.get_rmodel_complete().nv) {
    x_internal_.head<7>() = q.head<7>();
    x_internal_.segment<6>(designer_.get_rmodel().nq) = v.head<6>();

    int i = 0;
    for (unsigned long jointID : controlled_joints_ids_)
      if (jointID > 1) {
        x_internal_(i + 7) = q((long)jointID + 5);
        x_internal_(designer_.get_rmodel().nq + i + 6) = v((long)jointID + 4);
        i++;
      }
    return x_internal_;
  } else if (q.size() == designer_.get_rmodel().nq &&
             v.size() == designer_.get_rmodel().nv) {
    x_internal_ << q, v;
    return x_internal_;
  } else
    throw std::runtime_error(
        "q and v must have the dimentions of the reduced or complete model.");
}
}  // namespace deburring

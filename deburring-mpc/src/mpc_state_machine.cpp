#include "deburring_mpc/mpc.hpp"

namespace deburring {

void MPC::updateOCP() {
  switch (drilling_state_) {
    case Status::kInitialization:  // Initialization
      if (iteration_ < settings_.T_initialization) {
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = Status::kFirstHole;
      }
      break;

    case Status::kFirstHole:  // Approaching target
      if (iteration_ == 0) {
        std::cout << "Initiating movement" << std::endl;
      }
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeGoalCostActivation(OCP_.get_horizon_length() - iteration_,
                                      true);
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = Status::kStabilization;
      }
      break;

    case Status::kStabilization:  // Waiting for arm to stabilize (and closing
                                  // loop on MOCAP feedback)
      if (iteration_ == 0) {
        std::cout << "Waiting for arm to stabilize" << std::endl;
      }
      if (iteration_ < settings_.T_stabilization) {
        iteration_++;
      } else {
        iteration_ = 0;
        if (position_error_ > 0.05) {
          drilling_state_ = Status::kHoleTransition;
        } else {
          if (settings_.use_gain_scheduling == 1) {
            drilling_state_ = Status::kIncreaseGain;
          } else {
            drilling_state_ = Status::kDrilling;
          }
          // drilling_state_ = Status::kUpdatePosture;
        }
      }
      break;

    case Status::kIncreaseGain:  // Increasing gain
      if (iteration_ == 0) {
        std::cout << "Initiating increase gain phase" << std::endl;
      }
      if (position_error_ > settings_.precision_threshold &&
          goal_weight_ < settings_.gain_schedulig_max_weight) {
        goal_weight_ += settings_.gain_schedulig_slope;

        OCP_.changeGoalTrackingWeights(goal_weight_);

        iteration_++;
      } else {
        std::cout << "Gain Scheduling over" << std::endl;

        iteration_ = 0;
        drilling_state_ = Status::kDrilling;
      }
      break;

    case Status::kUpdatePosture:  // Update posture
      if (iteration_ == 0) {
        std::cout << "Updating reference posture" << std::endl;
      }
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changePostureReference(OCP_.get_horizon_length() - iteration_,
                                    Eigen::VectorXd::Zero(1));
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = Status::kDrilling;
      }
      break;

    case Status::kDrilling:  // Drilling
      if (iteration_ == 0) {
        std::cout << "Initiating drilling phase" << std::endl;
      }
      if (iteration_ <= settings_.T_drilling) {
        iteration_++;
      } else {
        std::cout << "Drilling over" << std::endl;

        iteration_ = 0;
        drilling_state_ = Status::kDisengagement;
      }
      break;

    case Status::kDisengagement:  // Getting out of hole
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
        drilling_state_ = Status::kHoleTransition;
      }
      break;

    case Status::kHoleTransition:  // Switching to next target
      if (iteration_ == 0) {
        if (current_hole_ < (number_holes_ - 1)) {
          current_hole_++;
          std::cout << "Switching to target: " << current_hole_ + 1
                    << std::endl;
          iteration_++;
        } else {
          std::cout << "Going back to inital position" << std::endl;
          drilling_state_ = Status::kReturnHome;
        }
      }
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeTarget(OCP_.get_horizon_length() - iteration_,
                          list_oMhole_[current_hole_].translation());
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = Status::kStabilization;
      }
      break;

    case Status::kReturnHome:  // Returning to initial position
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeGoalCostActivation(OCP_.get_horizon_length() - iteration_,
                                      false);
        iteration_++;
      } else {
        std::cout << "Robot is back to initial position" << std::endl;

        iteration_ = 0;
        drilling_state_ = Status::kDeburringDone;
      }
      break;

    case Status::kDeburringDone:  // Movement done
      break;
  }
}
}  // namespace deburring
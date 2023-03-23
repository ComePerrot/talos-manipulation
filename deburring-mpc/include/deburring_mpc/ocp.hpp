#ifndef OCP_P
#define OCP_P

#include <memory.h>

#include <pinocchio/fwd.hpp>
// include pinocchio first
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/quadratic-flat-log.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solver-base.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/frames.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "deburring_mpc/robot_designer.hpp"
#include "deburring_mpc/fwd.hpp"

namespace deburring {
// using namespace crocoddyl;

struct OCP_debugData {
 public:
  std::vector<Eigen::VectorXd> xi;
  std::vector<Eigen::VectorXd> ui;

  std::vector<Eigen::VectorXd> xs;
  std::vector<Eigen::VectorXd> us;
  std::vector<crocoddyl::SolverFDDP::MatrixXdRowMajor> K;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &xi;
    ar &ui;

    ar &xs;
    ar &us;
    ar &K;
  }
};

struct OCPSettings_Point {
  size_t horizon_length;
  // Timing
  double timeStep = 0.01;

  // Croco configuration
  double wStateReg = 0;
  double wControlReg = 0;
  double wLimit = 0;
  double wPCoM = 0;
  double wGripperPos = 0;
  double wGripperRot = 0;
  double wGripperVel = 0;

  double scaleLimits = 1;

  Eigen::VectorXd stateWeights;
  Eigen::VectorXd controlWeights;

  double th_stop = 1e-6;  // threshold for stopping criterion
  double th_grad = 1e-9;  // threshold for zero gradient.

  void readParamsFromYamlString(std::string &StringToParse);
  void readParamsFromYamlFile(const std::string &Filename);
};

class OCP_Point {
 private:
  OCPSettings_Point settings_;
  RobotDesigner designer_;
  DDP solver_;

  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;

  bool initialized_ = false;

  // prealocated memory:
  std::vector<VectorXd> warm_xs_;
  std::vector<VectorXd> warm_us_;

  // OCP Problem Maker private functions
  void buildSolver(const VectorXd x0, SE3 oMtarget);
  ActionModel formulatePointingTask();
  ActionModel formulateTerminalPointingTask();
  void setArmature(DifferentialActionModel DAM);
  void defineFeetContact(Contact &contactCollector);
  void definePostureTask(CostModelSum &costCollector, const double wStateReg);
  void defineActuationTask(CostModelSum &costCollector,
                           const double wControlReg);
  void defineJointLimits(CostModelSum &costCollector, const double wLimit,
                         const double boundScale);
  void defineCoMPosition(CostModelSum &costCollector, const double wPCoM);
  void defineGripperPlacement(CostModelSum &costCollector,
                              const double wGripperPos,
                              const double wGripperRot);
  void defineGripperVelocity(CostModelSum &costCollector,
                             const double wGripperVel);

  // OCP Problem Helper private functions
  ActionModel ama(const unsigned long time);
  IntegratedActionModel iam(const unsigned long time);
  DifferentialActionModel dam(const unsigned long time);
  CostModelSum costs(const unsigned long time);
  ActionData ada(const unsigned long time);

 public:
  OCP_Point(const OCPSettings_Point &OCPSettings,
            const RobotDesigner &designer);

  void initialize(const ConstVectorRef &x0, const SE3 &oMtarget);
  void solveFirst(const VectorXd x);
  void solve(const ConstVectorRef &measured_x);

  // OCP Problem Helper public functions
  void recede();
  void setBalancingTorques();
  void changeTarget(const size_t index,
                    const Eigen::Ref<const Eigen::Vector3d> &position);
  void updateGoalPosition(const Eigen::Ref<const Eigen::Vector3d> &position);
  void updateGoalRotation(const Eigen::Ref<const Eigen::Matrix3d> &rotation);
  void changeGoalCostActivation(const size_t index, const bool value);
  void changeGoaleTrackingWeights(double weight);
  void changePostureReference(const size_t index,
                              const Eigen::Ref<const VectorXd> reference);
  const VectorXd &getFinalPosture();

  // Debug
  void printCosts();

  std::vector<OCP_debugData> debugDataOCP_;

  void logData(const std::vector<Eigen::VectorXd> &x_init,
               const std::vector<Eigen::VectorXd> &u_init,
               const std::vector<Eigen::VectorXd> &xs,
               const std::vector<Eigen::VectorXd> &us,
               const std::vector<crocoddyl::SolverFDDP::MatrixXdRowMajor> &K);
  void dumpToFile(std::string fileName);
  std::vector<OCP_debugData> fetchFromFile(std::string fileName);
  void reprOCP(const unsigned long time);

  // Setters and Getters
  boost::shared_ptr<crocoddyl::StateMultibody> get_state() { return state_; }
  const VectorXd get_torque();
  const MatrixXd get_gain();

  OCPSettings_Point &get_settings() { return settings_; };
  DDP get_solver() { return (solver_); };
  size_t get_initialized() { return (initialized_); };
  size_t get_horizonLength() { return (settings_.horizon_length); };
};

}  // namespace deburring

#endif  // OCP_P

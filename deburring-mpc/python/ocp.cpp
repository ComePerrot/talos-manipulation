#include <pinocchio/multibody/fwd.hpp>
// #include <pinocchio/multibody/fwd.hpp>

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <eigenpy/eigenpy.hpp>

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/ocp.hpp"

namespace deburring {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

static boost::shared_ptr<OCP> constructor(bp::dict settings,
                                          const RobotDesigner &designer) {
  OCPSettings conf;
  conf.horizon_length = bp::extract<size_t>(settings["horizon_length"]);
  conf.time_step = bp::extract<double>(settings["time_step"]);

  conf.w_state_reg = bp::extract<double>(settings["w_state_reg"]);
  conf.w_control_reg = bp::extract<double>(settings["w_control_reg"]);
  conf.w_limit = bp::extract<double>(settings["w_limit"]);
  conf.w_com_pos = bp::extract<double>(settings["w_com_pos"]);
  conf.w_gripper_pos = bp::extract<double>(settings["w_gripper_pos"]);
  conf.w_gripper_rot = bp::extract<double>(settings["w_gripper_rot"]);
  conf.w_gripper_vel = bp::extract<double>(settings["w_gripper_vel"]);

  conf.limit_scale = bp::extract<double>(settings["limit_scale"]);

  conf.state_weights = bp::extract<Eigen::VectorXd>(settings["state_weights"]);
  conf.control_weights =
      bp::extract<Eigen::VectorXd>(settings["control_weights"]);

  return (boost::shared_ptr<OCP>(new OCP(conf, designer)));
}

void exposeOCPClass() {
  bp::register_ptr_to_python<boost::shared_ptr<OCP>>();

  bp::class_<OCP>("OCP", bp::no_init)
      .def("__init__", bp::make_constructor(&constructor))
      .def<void (OCP::*)(const ConstVectorRef &, const SE3 &)>(
          "initialize", &OCP::initialize, bp::args("self", "x0", "oMtarget"))
      .def<void (OCP::*)(const ConstVectorRef &, const SE3 &)>(
          "reset", &OCP::reset,
          bp::args("self", "x0", "oMtarget", "warm_xs", "warm_us"))
      .def<void (OCP::*)(const VectorXd)>("solve_first", &OCP::solveFirst,
                                          bp::args("self", "x"))
      .def<void (OCP::*)(const ConstVectorRef &)>(
          "solve", &OCP::solve, bp::args("self", "x_measured"))
      .def("recede", &OCP::recede)
      .def("change_goal_cost_activation", &OCP::changeGoalCostActivation,
           bp::args("index", "value"))
      .def("change_target", &OCP::changeTarget, bp::args("index", "position"))
      .def("change_goal_tracking_weights", &OCP::changeGoalTrackingWeights,
           bp::args("weight"))
      .def("change_posture_reference", &OCP::changePostureReference,
           bp::args("index", "reference"))
      .add_property("state", &OCP::get_state)
      .add_property("solver", &OCP::get_solver)
      .add_property("torque", &OCP::get_torque,
                    "Torque command computed by the OCP")
      .add_property("gain", &OCP::get_gain, "Gains computed by the OCP");
}

void exposeOCP() { exposeOCPClass(); }

}  // namespace python
}  // namespace deburring

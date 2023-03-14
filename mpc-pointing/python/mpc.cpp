#include <pinocchio/multibody/fwd.hpp>
// Must be included first!

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <eigenpy/eigenpy.hpp>

#include "mpc-pointing/fwd.hpp"
#include "mpc-pointing/mpc.hpp"

namespace mpc_p {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeMPCPointParams() {
  bp::register_ptr_to_python<boost::shared_ptr<MPCSettings_Point> >();

  bp::class_<MPCSettings_Point>(
      "MPCSettings_Point",
      bp::init<>(bp::args("self"), "Empty initialization of the MPC params"))
      .def("readFromYaml", &MPCSettings_Point::readParamsFromYamlFile,
           bp::args("filename"))
      .add_property("T_initialization",
                    bp::make_getter(&MPCSettings_Point::T_initialization),
                    bp::make_setter(&MPCSettings_Point::T_initialization),
                    "T_initialization.")
      .add_property("T_stabilization",
                    bp::make_getter(&MPCSettings_Point::T_stabilization),
                    bp::make_setter(&MPCSettings_Point::T_stabilization),
                    "T_stabilization.")
      .add_property(
          "T_drilling", bp::make_getter(&MPCSettings_Point::T_drilling),
          bp::make_setter(&MPCSettings_Point::T_drilling), "T_drilling.")
      .add_property("use_mocap", bp::make_getter(&MPCSettings_Point::use_mocap),
                    bp::make_setter(&MPCSettings_Point::use_mocap),
                    "use_mocap.")
      .add_property("use_gainScheduling",
                    bp::make_getter(&MPCSettings_Point::use_gainScheduling),
                    bp::make_setter(&MPCSettings_Point::use_gainScheduling),
                    "use_gainScheduling.")
      .add_property("gainSchedulig_slope",
                    bp::make_getter(&MPCSettings_Point::gainSchedulig_slope),
                    bp::make_setter(&MPCSettings_Point::gainSchedulig_slope),
                    "gainSchedulig_slope.")
      .add_property(
          "maxGoalWeight", bp::make_getter(&MPCSettings_Point::maxGoalWeight),
          bp::make_setter(&MPCSettings_Point::maxGoalWeight), "maxGoalWeight.")
      .add_property("targetPos", bp::make_getter(&MPCSettings_Point::targetPos),
                    bp::make_setter(&MPCSettings_Point::targetPos),
                    "targetPos.")
      .add_property(
          "holes_offsets", bp::make_getter(&MPCSettings_Point::holes_offsets),
          bp::make_setter(&MPCSettings_Point::holes_offsets), "holes_offsets.")
      .add_property("backwardOffset",
                    bp::make_getter(&MPCSettings_Point::backwardOffset),
                    bp::make_setter(&MPCSettings_Point::backwardOffset),
                    "backwardOffset.")
      .add_property("tolerance", bp::make_getter(&MPCSettings_Point::tolerance),
                    bp::make_setter(&MPCSettings_Point::tolerance),
                    "tolerance.");
}

void exposeMPCPointClass() {
  bp::register_ptr_to_python<boost::shared_ptr<MPC_Point> >();

  bp::class_<MPC_Point>(
      "MPC_Point", bp::init<const MPCSettings_Point &,
                            const OCPSettings_Point &, const RobotDesigner &>(
                       bp::args("self", "settings", "OCPSettings", "designer"),
                       "Initialize the MPC (empty init)"))
      .def<void (MPC_Point::*)(const ConstVectorRef &, const ConstVectorRef &,
                               const SE3 &)>(
          "initialize", &MPC_Point::initialize,
          bp::args("self", "q0", "v0", "toolMtarget"))
      .def<void (MPC_Point::*)(const ConstVectorRef &, const ConstVectorRef &,
                               const SE3 &)>(
          "iterate", &MPC_Point::iterate,
          bp::args("self", "q_current", "v_current", "toolMtarget"))
      .def<void (MPC_Point::*)(const VectorXd &, const SE3 &)>(
          "iterate", &MPC_Point::iterate, bp::args("self", "x0", "toolMtarget"))
      .add_property(
          "designer",
          bp::make_function(
              &MPC_Point::get_designer,
              bp::return_value_policy<bp::reference_existing_object>()),
          "pinocchio wrapper used by the MPC")
      .add_property(
          "OCP",
          bp::make_function(
              &MPC_Point::get_OCP,
              bp::return_value_policy<bp::reference_existing_object>()),
          "crocoddyl wrapper used by the MPC")
      .add_property(
          "oMtarget",
          bp::make_function(
              &MPC_Point::get_Target_frame,
              bp::return_value_policy<bp::reference_existing_object>()),
          "placement of the target in the robot frame")
      .add_property(
          "x0",
          bp::make_function(
              &MPC_Point::get_x0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "initial State")
      .add_property(
          "u0",
          bp::make_function(
              &MPC_Point::get_u0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "torque")
      .add_property(
          "K0",
          bp::make_function(
              &MPC_Point::get_K0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "Riccati gains")
      .add_property("drillingState", &MPC_Point::get_drillingState,
                    "Riccati gains");
}

void exposeMPCPoint() {
  exposeMPCPointParams();
  exposeMPCPointClass();
}

}  // namespace python
}  // namespace mpc_p

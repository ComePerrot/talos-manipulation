#include <pinocchio/multibody/fwd.hpp>
// Must be included first!

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <eigenpy/eigenpy.hpp>

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/mpc.hpp"

namespace deburring {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeMPCParams() {
  bp::register_ptr_to_python<boost::shared_ptr<MPCSettings> >();

  bp::class_<MPCSettings>(
      "MPCSettings",
      bp::init<>(bp::args("self"), "Empty initialization of the MPC params"))
      .def("read_from_yaml", &MPCSettings::readParamsFromYamlFile,
           bp::args("filename"))
      .add_property("T_initialization",
                    bp::make_getter(&MPCSettings::T_initialization),
                    bp::make_setter(&MPCSettings::T_initialization),
                    "T_initialization.")
      .add_property("T_stabilization",
                    bp::make_getter(&MPCSettings::T_stabilization),
                    bp::make_setter(&MPCSettings::T_stabilization),
                    "T_stabilization.")
      .add_property(
          "T_drilling", bp::make_getter(&MPCSettings::T_drilling),
          bp::make_setter(&MPCSettings::T_drilling), "T_drilling.")
      .add_property("use_mocap", bp::make_getter(&MPCSettings::use_mocap),
                    bp::make_setter(&MPCSettings::use_mocap),
                    "use_mocap.")
      .add_property("use_gain_scheduling",
                    bp::make_getter(&MPCSettings::use_gain_scheduling),
                    bp::make_setter(&MPCSettings::use_gain_scheduling),
                    "use_gain_scheduling.")
      .add_property("gain_schedulig_slope",
                    bp::make_getter(&MPCSettings::gain_schedulig_slope),
                    bp::make_setter(&MPCSettings::gain_schedulig_slope),
                    "gain_schedulig_slope.")
      .add_property(
          "gain_schedulig_max_weight", bp::make_getter(&MPCSettings::gain_schedulig_max_weight),
          bp::make_setter(&MPCSettings::gain_schedulig_max_weight), "gain_schedulig_max_weight.")
      .add_property("target_position", bp::make_getter(&MPCSettings::target_position),
                    bp::make_setter(&MPCSettings::target_position),
                    "target_position.")
      .add_property(
          "holes_offsets", bp::make_getter(&MPCSettings::holes_offsets),
          bp::make_setter(&MPCSettings::holes_offsets), "holes_offsets.")
      .add_property("backwardOffset",
                    bp::make_getter(&MPCSettings::backwardOffset),
                    bp::make_setter(&MPCSettings::backwardOffset),
                    "backwardOffset.")
      .add_property("precision_threshold", bp::make_getter(&MPCSettings::precision_threshold),
                    bp::make_setter(&MPCSettings::precision_threshold),
                    "precision_threshold.");
}

void exposeMPCClass() {
  bp::register_ptr_to_python<boost::shared_ptr<MPC> >();

  bp::class_<MPC>(
      "MPC", bp::init<const MPCSettings &,
                            const OCPSettings &, const RobotDesigner &>(
                       bp::args("self", "settings", "OCPSettings", "designer"),
                       "Initialize the MPC (empty init)"))
      .def<void (MPC::*)(const ConstVectorRef &, const ConstVectorRef &,
                               const SE3 &)>(
          "initialize", &MPC::initialize,
          bp::args("self", "q0", "v0", "toolMtarget"))
      .def<void (MPC::*)(const ConstVectorRef &, const ConstVectorRef &,
                               const SE3 &)>(
          "iterate", &MPC::iterate,
          bp::args("self", "q_current", "v_current", "toolMtarget"))
      .def<void (MPC::*)(const VectorXd &, const SE3 &)>(
          "iterate", &MPC::iterate, bp::args("self", "x0", "toolMtarget"))
      .add_property(
          "designer",
          bp::make_function(
              &MPC::get_designer,
              bp::return_value_policy<bp::reference_existing_object>()),
          "pinocchio wrapper used by the MPC")
      .add_property(
          "OCP",
          bp::make_function(
              &MPC::get_OCP,
              bp::return_value_policy<bp::reference_existing_object>()),
          "crocoddyl wrapper used by the MPC")
      .add_property(
          "oMtarget",
          bp::make_function(
              &MPC::get_Target_frame,
              bp::return_value_policy<bp::reference_existing_object>()),
          "placement of the target in the robot frame")
      .add_property(
          "x0",
          bp::make_function(
              &MPC::get_x0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "initial State")
      .add_property(
          "u0",
          bp::make_function(
              &MPC::get_u0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "torque")
      .add_property(
          "K0",
          bp::make_function(
              &MPC::get_K0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "Riccati gains")
      .add_property("drillingState", &MPC::get_drillingState,
                    "Riccati gains");
}

void exposeMPC() {
  exposeMPCParams();
  exposeMPCClass();
}

}  // namespace python
}  // namespace deburring

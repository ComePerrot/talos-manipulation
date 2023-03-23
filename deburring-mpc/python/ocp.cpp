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

void exposeOCPPointParams() {
  bp::register_ptr_to_python<boost::shared_ptr<OCPSettings>>();

  bp::class_<OCPSettings>(
      "OCPSettings",
      bp::init<>(bp::args("self"), "Empty initialization of the OCP params"))
      .def("readFromYaml", &OCPSettings::readParamsFromYamlFile,
           bp::args("filename"))
      .add_property("horizon_length",
                    bp::make_getter(&OCPSettings::horizon_length),
                    bp::make_setter(&OCPSettings::horizon_length),
                    "Number of nodes in the horizon");
}

void exposeOCPPointClass() {
  bp::register_ptr_to_python<boost::shared_ptr<OCP>>();

  bp::class_<OCP>(
      "OCP",
      bp::init<const OCPSettings &, const RobotDesigner &>(
          bp::args("self", "OCPSettings", "designer"),
          "Initialize the OCP from parameter list and a robot designer"))
      .def<void (OCP::*)(const ConstVectorRef &, const SE3 &)>(
          "initialize", &OCP::initialize,
          bp::args("self", "x0", "oMtarget"))
      .def<void (OCP::*)(const VectorXd)>(
          "solveFirst", &OCP::solveFirst, bp::args("self", "x"))
      .def<void (OCP::*)(const ConstVectorRef &)>(
          "solve", &OCP::solve, bp::args("self", "x_measured"))
      .def("changeGoalCostActivation", &OCP::changeGoalCostActivation,
           bp::args("index", "value"))
      .def("changeTarget", &OCP::changeTarget,
           bp::args("index", "position"))
      .def("changeGoalTrackingWeights", &OCP::changeGoaleTrackingWeights,
           bp::args("weight"))
      .def("changePostureReference", &OCP::changePostureReference,
           bp::args("index", "reference"))
      .add_property("state", &OCP::get_state)
      .add_property("solver", &OCP::get_solver)
      .add_property("torque", &OCP::get_torque,
                    "Torque command computed by the OCP")
      .add_property("gain", &OCP::get_gain, "Gains computed by the OCP");
}

void exposeOCPPoint() {
  exposeOCPPointParams();
  exposeOCPPointClass();
}

}  // namespace python
}  // namespace deburring

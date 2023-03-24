#include <pinocchio/multibody/fwd.hpp>  // Must be included first!
// This line must be the first include
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "pinocchio/multibody/model.hpp"
#include "deburring_mpc/robot_designer.hpp"

namespace deburring {
namespace python {
namespace bp = boost::python;

template <typename T>
inline void py_list_to_std_vector(const bp::object &iterable,
                                  std::vector<T> &out) {
  out = std::vector<T>(boost::python::stl_input_iterator<T>(iterable),
                       boost::python::stl_input_iterator<T>());
}

void initialize(RobotDesigner &self, bp::dict settings) {
  RobotDesignerSettings conf;
  conf.urdf_path = bp::extract<std::string>(settings["urdf_path"]);
  conf.srdf_path = bp::extract<std::string>(settings["srdf_path"]);
  conf.left_foot_name = bp::extract<std::string>(settings["left_foot_name"]);
  conf.right_foot_name = bp::extract<std::string>(settings["right_foot_name"]);
  conf.robot_description =
      bp::extract<std::string>(settings["robot_description"]);
  py_list_to_std_vector(settings["controlled_joints_names"],
                        conf.controlled_joints_names);

  self.initialize(conf);
}

bp::dict get_settings(RobotDesigner &self) {
  RobotDesignerSettings conf = self.get_settings();
  bp::dict settings;
  settings["urdf_path"] = conf.urdf_path;
  settings["srdf_path"] = conf.srdf_path;
  settings["left_foot_name"] = conf.left_foot_name;
  settings["right_foot_name"] = conf.right_foot_name;
  settings["robot_description"] = conf.robot_description;

  return settings;
}

pinocchio::Model get_rmodel_complete(RobotDesigner &self) {
  return self.get_rmodel_complete();
}

// pinocchio::Model get_rmodel(RobotDesigner &self) { return self.get_rmodel();
// }

pinocchio::Data get_rdata(RobotDesigner &self) { return self.get_rdata(); }

pinocchio::Data get_rdata_complete(RobotDesigner &self) {
  return self.get_rdata_complete();
}

void exposeRobotDesigner() {
  bp::class_<RobotDesigner>("RobotDesigner", bp::init<>())
      .def("initialize", &initialize)
      .def("update_reduced_model", &RobotDesigner::updateReducedModel)
      .def("update_complete_model", &RobotDesigner::updateCompleteModel)
      .def("add_end_effector_frame", &RobotDesigner::addEndEffectorFrame)
      .def("get_lf_frame",
           bp::make_function(
               &RobotDesigner::get_lf_frame,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_end_effector_frame",
           bp::make_function(
               &RobotDesigner::get_end_effector_frame,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_rf_frame",
           bp::make_function(
               &RobotDesigner::get_rf_frame,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_robot_mass", &RobotDesigner::get_robot_mass)
      .def("get_rmodel",
           bp::make_function(
               &RobotDesigner::get_rmodel,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_rmodel_complete",
           bp::make_function(
               &RobotDesigner::get_rmodel_complete,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_rdata",
           bp::make_function(
               &RobotDesigner::get_rdata,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_rdata_complete",
           bp::make_function(
               &RobotDesigner::get_rdata_complete,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_q0",
           bp::make_function(
               &RobotDesigner::get_q0,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_q0_complete",
           bp::make_function(
               &RobotDesigner::get_q0_complete,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_v0",
           bp::make_function(
               &RobotDesigner::get_v0,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_v0_complete",
           bp::make_function(
               &RobotDesigner::get_v0_complete,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_x0",
           bp::make_function(
               &RobotDesigner::get_x0,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_lf_name",
           bp::make_function(
               &RobotDesigner::get_lf_name,
               bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_rf_name",
           bp::make_function(
               &RobotDesigner::get_rf_name,
               bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_lf_id",
           bp::make_function(
               &RobotDesigner::get_lf_id,
               bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_rf_id",
           bp::make_function(
               &RobotDesigner::get_rf_id,
               bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_com_position",
           bp::make_function(
               &RobotDesigner::get_com_position,
               bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_end_effector_id",
           bp::make_function(
               &RobotDesigner::get_end_effector_id,
               bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_settings",
           bp::make_function(
               &RobotDesigner::get_settings,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_controlled_joints_ids",
           bp::make_function(
               &RobotDesigner::get_controlled_joints_ids,
               bp::return_value_policy<bp::copy_const_reference>()));

  return;
}

}  // namespace python
}  // namespace deburring
/**
 * This file follows the same syntax as registration_utils.h of the
 * pal_statistics package.
 *
 * It implements functions to register custom types using PAL's architecture.
 */

#ifndef CUSTOM_REGISTRATION
#define CUSTOM_REGISTRATION

#include <pal_statistics/pal_statistics_macros.h>

#include <Eigen/Dense>
#include <pinocchio/spatial/fwd.hpp>

namespace pal_statistics {
/**
 * @brief Implementation for Eigen::Vector3d
 */
IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                      const Eigen::Vector3d *variable,
                      RegistrationsRAII *bookkeeping = NULL,
                      bool enabled = true) {
  registry.registerVariable(name + "_x", &variable->x(), bookkeeping, enabled);
  registry.registerVariable(name + "_y", &variable->y(), bookkeeping, enabled);
  return registry.registerVariable(name + "_z", &variable->z(), bookkeeping,
                                   enabled);
}

/**
 * @brief Implementation for Eigen::Quaterniond
 */
IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                      const Eigen::Quaterniond *variable,
                      RegistrationsRAII *bookkeeping = NULL,
                      bool enabled = true) {
  registry.registerVariable(name + "_qx", &variable->coeffs().x(), bookkeeping,
                            enabled);
  registry.registerVariable(name + "_qy", &variable->coeffs().y(), bookkeeping,
                            enabled);
  registry.registerVariable(name + "_qz", &variable->coeffs().z(), bookkeeping,
                            enabled);
  return registry.registerVariable(name + "_qw", &variable->coeffs().w(),
                                   bookkeeping, enabled);
}

// /**
//  * @brief Implementation for pinocchio::SE3
//  */
// IdType customRegister(StatisticsRegistry &registry, const std::string &name,
//                       const pinocchio::SE3 *variable,
//                       RegistrationsRAII *bookkeeping = NULL,
//                       bool enabled = true) {
//   registry.registerVariable(name + "position_x",
//   &variable->translation().x(),
//                             bookkeeping, enabled);
//   registry.registerVariable(name + "position_y",
//   &variable->translation().y(),
//                             bookkeeping, enabled);
//   registry.registerVariable(name + "position_z",
//   &variable->translation().z(),
//                             bookkeeping, enabled);

//   registry.registerVariable(name + "rotation_qx",
//                             &variable->rotation().coeffs().x(), bookkeeping,
//                             enabled);
//   registry.registerVariable(name + "rotation_qy",
//                             &variable->rotation().coeffs().y(), bookkeeping,
//                             enabled);
//   registry.registerVariable(name + "rotation_qz",
//                             &variable->rotation().coeffs().z(), bookkeeping,
//                             enabled);
//   return registry.registerVariable(name + "rotation_qw",
//                                    &variable->rotation().coeffs().w(),
//                                    bookkeeping, enabled);
// }
}  // namespace pal_statistics

#endif  // CUSTOM_REGISTRATION
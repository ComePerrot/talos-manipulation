

#ifndef DEBURRING_ROBOT_DESIGNER_FACTORY
#define DEBURRING_ROBOT_DESIGNER_FACTORY

#include "deburring_mpc/robot_designer.hpp"

namespace deburring {
namespace unittest {

class RobotDesignerFactory {
 public:
  deburring::RobotDesigner createRobotDesigner();
};
}  // namespace unittest
}  // namespace deburring

#endif  // DEBURRING_ROBOT_DESIGNER_FACTORY
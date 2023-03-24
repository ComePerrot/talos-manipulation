#ifndef __deburring_python__
#define __deburring_python__

#include <boost/python.hpp>
#include <pinocchio/fwd.hpp>

namespace deburring {
namespace python {

void exposeRobotDesigner();
void exposeOCP();
void exposeMPC();

}  // namespace python
}  // namespace deburring

#endif  // #ifndef __deburring_python__

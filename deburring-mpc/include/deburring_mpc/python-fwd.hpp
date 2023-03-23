#ifndef __mpc_p_python__
#define __mpc_p_python__

#include <boost/python.hpp>
#include <pinocchio/fwd.hpp>

namespace deburring {
namespace python {

void exposeDesigner();
void exposeOCPPoint();
void exposeMPCPoint();

}  // namespace python
}  // namespace deburring

#endif  // #ifndef __mpc_p_python__

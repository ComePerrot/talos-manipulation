#ifndef __mpc_p_python__
#define __mpc_p_python__

#include <boost/python.hpp>
#include <pinocchio/fwd.hpp>

namespace mpc_p {
namespace python {

void exposeDesigner();
void exposeOCPPoint();
void exposeMPCPoint();

}  // namespace python
}  // namespace mpc_p

#endif  // #ifndef __mpc_p_python__

#include "mpc-pointing/ocp.hpp"

namespace mpc_p {

void OCP_Point::logData(
    const std::vector<Eigen::VectorXd>& x_init,
    const std::vector<Eigen::VectorXd>& u_init,
    const std::vector<Eigen::VectorXd>& xs,
    const std::vector<Eigen::VectorXd>& us,
    const std::vector<crocoddyl::SolverFDDP::MatrixXdRowMajor>& K) {
  OCP_debugData data;
  data.xi = x_init;
  data.ui = u_init;

  data.xs = xs;
  data.us = us;
  data.K = K;

  debugDataOCP_.push_back(data);
}

void OCP_Point::dumpToFile(std::string name) {
  std::ofstream ofs(name.c_str());
  boost::archive::text_oarchive oa(ofs);
  oa << debugDataOCP_;
}

std::vector<OCP_debugData> OCP_Point::fetchFromFile(std::string name) {
  std::vector<OCP_debugData> datas;

  std::ifstream ifs(name.c_str());
  boost::archive::text_iarchive ia(ifs);
  ia >> datas;

  return (datas);
}

void OCP_Point::reprOCP(){
  auto modelCosts = costs(0)->get_costs();

  for (auto cost: modelCosts){
    std::cout << cost.second->name << cost.second->active << cost.second->weight << std::endl;
  }
}

}  // namespace mpc_p
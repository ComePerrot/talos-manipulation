#include "deburring_mpc/ocp.hpp"

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

void OCP_Point::reprOCP(const unsigned long time) {
  auto modelContacts = dam(time)->get_contacts()->get_contacts();

  // Contacts
  for (auto contact : modelContacts) {
    auto second = contact.second;

    std::cout << second->name << std::endl;
    std::cout << "  active: " << second->active << std::endl;
    std::cout << "  ";
    second->contact->print(std::cout);
    std::cout << std::endl;
  }

  // Costs
  auto modelCosts = costs(time)->get_costs();

  for (auto cost : modelCosts) {
    auto second = cost.second;

    std::cout << cost.second->name << std::endl;
    std::cout << "  active: " << cost.second->active << std::endl;
    std::cout << "  weight: " << cost.second->weight << std::endl;

    std::cout << "  ";
    second->cost->print(std::cout);
    std::cout << std::endl;

    // Residual
    auto residual = second->cost->get_residual();

    // actuation task
    auto actuationResidual =
        boost::dynamic_pointer_cast<crocoddyl::ResidualModelControl>(residual);
    if (actuationResidual != NULL) {
      std::cout << "    Residual reference: ";
      std::cout << actuationResidual->get_reference().transpose() << std::endl;
    }

    // posture task
    auto postureResidual =
        boost::dynamic_pointer_cast<crocoddyl::ResidualModelState>(residual);
    if (postureResidual != NULL) {
      std::cout << "    Residual reference: ";
      std::cout << postureResidual->get_reference().transpose() << std::endl;
    }

    // Activation
    auto activation = second->cost->get_activation();

    auto weightedQuad =
        boost::dynamic_pointer_cast<crocoddyl::ActivationModelWeightedQuad>(
            activation);
    if (weightedQuad != NULL) {
      std::cout << "    Activation weights: ";
      std::cout << weightedQuad->get_weights().transpose() << std::endl;
    }

    auto quadBarrier =
        boost::dynamic_pointer_cast<crocoddyl::ActivationModelQuadraticBarrier>(
            activation);
    if (quadBarrier != NULL) {
      std::cout << "    beta: ";
      std::cout << quadBarrier->get_bounds().beta << std::endl;
      std::cout << "    lower bound: ";
      std::cout << quadBarrier->get_bounds().lb.transpose() << std::endl;
      std::cout << "    upper bound: ";
      std::cout << quadBarrier->get_bounds().ub.transpose() << std::endl;
    }
  }
}

}  // namespace mpc_p
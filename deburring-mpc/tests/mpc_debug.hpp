#ifndef DEBURRING_MPC_DEBUG
#define DEBURRING_MPC_DEBUG

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/mpc.hpp"

namespace deburring {

struct MPCDebugData {
 public:
  Eigen::VectorXd x;

  Eigen::VectorXd u;
  Eigen::MatrixXd K;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &x;
    ar &u;
    ar &K;
  }
};

class MPCDebug : public MPC {
 private:
  std::vector<MPCDebugData> debug_data_mpc_;

 public:
  void logData(const Eigen::Ref<const Eigen::VectorXd> x_input,
               const Eigen::Ref<const Eigen::VectorXd> u_output,
               const Eigen::Ref<const Eigen::MatrixXd> K_output) {
    MPCDebugData data;
    data.x = x_input;
    data.u = u_output;
    data.K = K_output;

    debug_data_mpc_.push_back(data);
  }

  void dumpToFile(std::string name) {
    std::ofstream ofs(name.c_str());
    boost::archive::text_oarchive oa(ofs);
    oa << debug_data_mpc_;
  }

  std::vector<MPC_debugData> fetchFromFile(std::string name) {
    std::vector<MPC_debugData> datas;

    std::ifstream ifs(name.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> datas;

    return (datas);
  }

  void test(const VectorXd &x0) {
    this->iterate(x0, pinocchio::SE3::Identity());
    logData(x0, this->get_u0(), this->get_K0());
  }
};

}  // namespace deburring

#endif  // DEBURRING_MPC_DEBUG
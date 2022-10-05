#include <yaml-cpp/yaml.h>

#include "mpc-pointing/ocp.hpp"

namespace mpc_p {

void OCPSettings_Point::readParamsFromYamlString(std::string &StringToParse) {
  YAML::Node root = YAML::Load(StringToParse);
  YAML::Node config = root["ocp-point"];

  if (!config) {
    std::cerr << "No ocp-point section." << std::endl;
    return;
  }

  // Local lambda function to read size_t
  auto read_size_t = [&config](size_t &aref_size_t, std::string fieldname) {
    YAML::Node yn_asize_t = config[fieldname];
    if (yn_asize_t) {
      aref_size_t = yn_asize_t.as<size_t>();
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  // Local lambda function to read double
  auto read_double = [&config](double &aref_d, std::string fieldname) {
    YAML::Node yn_ad = config[fieldname];
    if (yn_ad) {
      aref_d = yn_ad.as<double>();
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  // Local lambda function to read VectorXd
  auto read_vxd = [&config](VectorXd &aref_vxd, std::string fieldname) {
    YAML::Node yn_avxd = config[fieldname];
    if (yn_avxd) {
      aref_vxd.resize((Eigen::Index)yn_avxd.size());
      for (std::size_t id = 0; id < yn_avxd.size(); id++) {
        aref_vxd[(Eigen::Index)id] = yn_avxd[id].as<double>();
      }
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  read_size_t(horizon_length, "horizon_length");

  read_double(modelMakerSettings.wStateReg, "wStateReg");
  read_double(modelMakerSettings.wControlReg, "wControlReg");
  read_double(modelMakerSettings.wLimit, "wLimit");
  read_double(modelMakerSettings.wPCoM, "wPCoM");
  read_double(modelMakerSettings.wVCoM, "wVCoM");
  read_double(modelMakerSettings.wWrenchCone, "wWrenchCone");
  read_double(modelMakerSettings.wGripperPos, "wGripperPos");
  read_double(modelMakerSettings.wGripperRot, "wGripperRot");
  read_double(modelMakerSettings.wGripperVel, "wGripperVel");

  read_vxd(modelMakerSettings.stateWeights, "stateWeights");
  read_vxd(modelMakerSettings.controlWeights, "controlWeights");
}

void OCPSettings_Point::readParamsFromYamlFile(const std::string &Filename) {
  std::ifstream t(Filename);
  std::stringstream buffer;
  buffer << t.rdbuf();
  std::string StringToParse = buffer.str();
  readParamsFromYamlString(StringToParse);
}

}  // namespace mpc_p
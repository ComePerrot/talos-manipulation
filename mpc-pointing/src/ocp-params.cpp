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
  auto read_vxd = [](VectorXd &aref_vxd, YAML::Node node,
                     std::string fieldname) {
    YAML::Node yn_avxd = node[fieldname];
    if (yn_avxd) {
      aref_vxd.resize((Eigen::Index)yn_avxd.size());
      for (std::size_t id = 0; id < yn_avxd.size(); id++) {
        aref_vxd[(Eigen::Index)id] = yn_avxd[id].as<double>();
      }
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  auto read_stateWeights = [&config, &read_vxd](VectorXd &aref_stateWeights) {
    std::array<std::string, 2> nodeNames{"statePosWeights", "stateVelWeights"};
    std::array<std::string, 6> limbs{"base",  "leftLeg", "rightLeg",
                                     "torso", "leftArm", "rightArm"};
    VectorXd stateWeights(36 * 2);  // Maximum size for the state with Talos

    int sizeWeight = 0;
    for (auto nodeName : nodeNames) {
      YAML::Node node = config[nodeName];
      if (node) {
        for (auto limb : limbs) {
          VectorXd buffer;
          read_vxd(buffer, node, limb);

          stateWeights.segment(sizeWeight, buffer.size()) = buffer;
          sizeWeight += (int)buffer.size();
        }
      } else {
        std::cout << "No " << nodeName << std::endl;
      }
    }
    std::cout << sizeWeight << std::endl;
    aref_stateWeights.resize((Eigen::Index)sizeWeight);
    aref_stateWeights = stateWeights.head(sizeWeight);
  };

  auto read_controlWeights = [&config,
                              &read_vxd](VectorXd &aref_controlWeights) {
    std::array<std::string, 5> limbs{"leftLeg", "rightLeg", "torso", "leftArm",
                                     "rightArm"};
    VectorXd controlWeights(36);

    int sizeWeight = 0;
    YAML::Node node = config["controlWeights"];
    if (node) {
      for (auto limb : limbs) {
        VectorXd buffer;
        read_vxd(buffer, node, limb);

        controlWeights.segment(sizeWeight, buffer.size()) = buffer;
        sizeWeight += (int)buffer.size();
      }
    } else {
      std::cout << "No controlWeights" << std::endl;
    }
    aref_controlWeights.resize((Eigen::Index)sizeWeight);
    aref_controlWeights = controlWeights.head(sizeWeight);
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

  read_stateWeights(modelMakerSettings.stateWeights);
  read_controlWeights(modelMakerSettings.controlWeights);
}

void OCPSettings_Point::readParamsFromYamlFile(const std::string &Filename) {
  std::ifstream t(Filename);
  std::stringstream buffer;
  buffer << t.rdbuf();
  std::string StringToParse = buffer.str();
  readParamsFromYamlString(StringToParse);
}

}  // namespace mpc_p
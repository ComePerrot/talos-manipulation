#include <yaml-cpp/yaml.h>

#include "deburring_mpc/ocp.hpp"

namespace deburring {

void OCPSettings::readParamsFromYamlString(const std::string &string_to_parse) {
  YAML::Node root = YAML::Load(string_to_parse);
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
    VectorXd state_weights(36 * 2);  // Maximum size for the state with Talos

    int sizeWeight = 0;
    for (auto nodeName : nodeNames) {
      YAML::Node node = config[nodeName];
      if (node) {
        for (auto limb : limbs) {
          VectorXd buffer;
          read_vxd(buffer, node, limb);

          state_weights.segment(sizeWeight, buffer.size()) = buffer;
          sizeWeight += (int)buffer.size();
        }
      } else {
        std::cout << "No state_weights" << std::endl;
      }
    }
    aref_stateWeights.resize((Eigen::Index)sizeWeight);
    aref_stateWeights = state_weights.head(sizeWeight);
  };

  auto read_controlWeights = [&config,
                              &read_vxd](VectorXd &aref_controlWeights) {
    std::array<std::string, 5> limbs{"leftLeg", "rightLeg", "torso", "leftArm",
                                     "rightArm"};
    VectorXd control_weights(36);

    int sizeWeight = 0;
    YAML::Node node = config["control_weights"];
    if (node) {
      for (auto limb : limbs) {
        VectorXd buffer;
        read_vxd(buffer, node, limb);

        control_weights.segment(sizeWeight, buffer.size()) = buffer;
        sizeWeight += (int)buffer.size();
      }
    } else {
      std::cout << "No control_weights" << std::endl;
    }
    aref_controlWeights.resize((Eigen::Index)sizeWeight);
    aref_controlWeights = control_weights.head(sizeWeight);
  };

  read_size_t(horizon_length, "horizon_length");

  read_double(w_state_reg, "w_state_reg");
  read_double(w_control_reg, "w_control_reg");
  read_double(w_limit, "w_limit");
  read_double(w_com_pos, "w_com_pos");
  read_double(w_gripper_pos, "w_gripper_pos");
  read_double(w_gripper_rot, "w_gripper_rot");
  read_double(w_gripper_vel, "w_gripper_vel");
  read_double(limit_scale, "limit_scale");

  read_stateWeights(state_weights);
  read_controlWeights(control_weights);
}

void OCPSettings::readParamsFromYamlFile(const std::string &filename) {
  std::ifstream t(filename);
  std::stringstream buffer;
  buffer << t.rdbuf();
  std::string StringToParse = buffer.str();
  readParamsFromYamlString(StringToParse);
}

}  // namespace deburring
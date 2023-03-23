#include <yaml-cpp/yaml.h>

#include "deburring_mpc/mpc.hpp"

namespace deburring {

void MPCSettings_Point::readParamsFromYamlString(std::string &StringToParse) {
  YAML::Node root = YAML::Load(StringToParse);
  YAML::Node config = root["mpc-point"];

  if (!config) {
    std::cerr << "No mpc-point section." << std::endl;
    return;
  }

  // Local lambda function to read int
  auto read_int = [&config](int &aref_int, std::string fieldname) {
    YAML::Node yn_aint = config[fieldname];
    if (yn_aint) {
      aref_int = yn_aint.as<int>();
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

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

  // Local lambda function to read vector3
  auto read_v3d = [&config](Vector3d &aref_v3d, std::string fieldname) {
    YAML::Node yn_av3d = config[fieldname];
    if (yn_av3d) {
      aref_v3d.resize(3);
      for (std::size_t id = 0; id < yn_av3d.size(); id++) {
        aref_v3d[(Eigen::Index)id] = yn_av3d[id].as<double>();
      }
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  // Local lambda function to read std::vector<Eigen::Vector3d>
  auto read_stdvect_v3d = [&config](
                              std::vector<Vector3d> &aref_stdvect_v3d,
                              std::string fieldname) {
    YAML::Node yn_astdvect_v3d = config[fieldname];
    if (yn_astdvect_v3d) {
      for (std::size_t i = 0; i < yn_astdvect_v3d.size(); i++) {
        Vector3d buffer;
        for (std::size_t j = 0; j < 3; j++) {
          buffer[(Eigen::Index)j] = yn_astdvect_v3d[i][j].as<double>();
        }
        aref_stdvect_v3d.push_back(buffer);
      }
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  read_size_t(T_initialization, "T_initialization");
  read_size_t(T_stabilization, "T_stabilization");
  read_size_t(T_drilling, "T_drilling");
  read_int(use_mocap, "use_mocap");
  read_int(use_gainScheduling, "use_gainScheduling");
  read_double(gainSchedulig_slope, "gainSchedulig_slope");
  read_double(maxGoalWeight, "maxGoalWeight");
  read_v3d(targetPos, "targetPos");
  read_stdvect_v3d(holes_offsets, "holes_offsets");
  read_double(tolerance, "tolerance");
}

void MPCSettings_Point::readParamsFromYamlFile(const std::string &Filename) {
  std::ifstream t(Filename);
  std::stringstream buffer;
  buffer << t.rdbuf();
  std::string StringToParse = buffer.str();
  readParamsFromYamlString(StringToParse);
}

}  // namespace deburring

#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include "common.hpp"
#include "deburring_mpc/mpc.hpp"
#include "factory/robot_designer.hpp"

using namespace boost::unit_test;

void testMPC() {
  // Robot Designer Factory
  deburring::unittest::RobotDesignerFactory factory;
  deburring::RobotDesigner pinWrapper = factory.createRobotDesigner();

  //  MPC
  //    OCP parameters
  std::string parameterFile(PROJECT_SOURCE_DIR
                            "/tests/archive/"
                            "test_settings.yaml");
  deburring::OCPSettings ocpSettings = deburring::OCPSettings();
  ocpSettings.readParamsFromYamlFile(parameterFile);
  //    MPC parameters
  deburring::MPCSettings mpcSettings = deburring::MPCSettings();
  mpcSettings.readParamsFromYamlFile(parameterFile);

  deburring::MPC MPC = deburring::MPC(mpcSettings, ocpSettings, pinWrapper);

  // Load data from serialized file
  std::vector<deburring::MPC_debugData>::size_type iteration = 0;
  std::vector<deburring::MPC_debugData> debugData =
      MPC.fetchFromFile(PROJECT_SOURCE_DIR
                        "/tests/archive/"
                        "data_MPC.txt");

  // Initialize OCP with the same initial state
  Eigen::VectorXd x_current = debugData[iteration].x_input;

  MPC.initialize(x_current.head(MPC.get_designer().get_rmodel().nq),
                 x_current.tail(MPC.get_designer().get_rmodel().nv),
                 pinocchio::SE3::Identity());

  // Compare loaded data with the one generated by the OCP
  deburring::MPC_command output;

  while (iteration < debugData.size() - 1) {
    output = debugData[iteration].output;

    auto us_MPC = MPC.get_u0();
    auto us_Data = output.us0;

    // Riccati
    auto K_OCP = MPC.get_K0();
    auto K_Data = output.K0;

    BOOST_CHECK((us_MPC - us_Data).isZero(1e-6));
    BOOST_CHECK((K_OCP - K_Data).isZero(1e-4));

    iteration++;

    MPC.iterate(debugData[iteration].x_input, pinocchio::SE3::Identity());
  }
}

void registerMPCUnitTest() {
  test_suite* ts = BOOST_TEST_SUITE("test_MPC");
  ts->add(BOOST_TEST_CASE(testMPC));
  framework::master_test_suite().add(ts);
}

bool init_function() {
  registerMPCUnitTest();
  return true;
}

int main(int argc, char** argv) {
  return ::boost::unit_test::unit_test_main(&init_function, argc, argv);
}

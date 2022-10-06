#####################
#  LOADING MODULES  #
#####################

import pinocchio as pin

from sobec import RobotDesigner
from mpc_pointing import MPC_Point, MPCSettings_Point, OCPSettings_Point

from TalosDeburringSimulation import TalosDeburringSimulator

################
#  PARAMETERS  #
################

modelPath = "/opt/openrobots/share/example-robot-data/robots/talos_data/"
URDF = modelPath + "robots/talos_reduced.urdf"
SRDF = modelPath + "srdf/talos.srdf"
controlledJoints = [
    "root_joint",
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",
    "leg_left_6_joint",
    "leg_right_1_joint",
    "leg_right_2_joint",
    "leg_right_3_joint",
    "leg_right_4_joint",
    "leg_right_5_joint",
    "leg_right_6_joint",
    "torso_1_joint",
    "torso_2_joint",
    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
]

####################
#  INITIALIZATION  #
####################

# Robot model
design_conf = dict(
    urdfPath=URDF,
    srdfPath=SRDF,
    leftFootName="right_sole_link",
    rightFootName="left_sole_link",
    robotDescription="",
    controlledJointsNames=controlledJoints,
)
pinWrapper = RobotDesigner()
pinWrapper.initialize(design_conf)

gripper_SE3_tool = pin.SE3.Identity()
gripper_SE3_tool.translation[0] = 0
gripper_SE3_tool.translation[1] = -0.02
gripper_SE3_tool.translation[2] = -0.0825
pinWrapper.addEndEffectorFrame(
    "deburring_tool", "gripper_left_fingertip_3_link", gripper_SE3_tool
)

# Simulator

q0 = pinWrapper.get_rModelComplete().referenceConfigurations["half_sitting"]

simulator = TalosDeburringSimulator(
    URDF=URDF,
    targetPos=[0.6, 0.4, 1.1],
    rmodelComplete=pinWrapper.get_rModelComplete(),
    controlledJointsIDs=pinWrapper.get_controlledJointsIDs(),
    enableGUI=True,
)

# MPC
#   Loading parameters from file
filename = "/local/users/cperrot/pointing_controller/config/Settings_sobec.yaml"

MPCparams = MPCSettings_Point()
OCPparams = OCPSettings_Point()

OCPparams.readFromYaml(filename)
MPCparams.readFromYaml(filename)

print("Parameters loaded from file: " + filename)

#   MPC
MPC = MPC_Point(MPCparams, OCPparams, pinWrapper)
MPC.initialize(pinWrapper.get_q0(), pinWrapper.get_v0(), pin.SE3.Identity())

state = MPC.OCP.modelMaker.getState()

###############
#  MAIN LOOP  #
###############
NcontrolKnots = 10

while True:
    # Controller works faster than trajectory generation
    for i_control in range(NcontrolKnots):
        x_measured = simulator.getRobotState()

        # Compute torque to be applied by adding Riccati term
        torques = MPC.u0 + MPC.K0 @ state.diff(x_measured, MPC.x0)

        # Apply torque on complete model
        simulator.step(torques)

    MPC.iterate(x_measured, pin.SE3.Identity())

simulator.end()

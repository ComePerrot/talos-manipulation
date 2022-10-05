#####################
#  LOADING MODULES  #
#####################

import numpy as np
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

simulator = TalosDeburringSimulator(
    URDF_File=URDF,
    URDF_Path="/talos_data/robots/",
    targetPos=[0.6, 0.4, 1.1],
    rmodel=pinWrapper.get_rModelComplete(),
    ControlledJoints=controlledJoints,
    initialConfiguration=pinWrapper.get_q0(),
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
        q_current, v_current = simulator.getCurrenRobotState()

        # Compute current position and velocity of all crocoddyl joints
        qc = np.vstack(
            (q_current[[i + 5 for i in pinWrapper.get_controlledJointsIDs()[1:]]])
        )
        vc = np.vstack(
            (v_current[[i + 4 for i in pinWrapper.get_controlledJointsIDs()[1:]]])
        )

        xinit0 = np.vstack((q_current[:7], qc, v_current[:6], vc))
        xinit0[:3] -= simulator.localInertiaPos

        # Compute torque to be applied by adding Riccati term
        torques = MPC.u0 + MPC.K0 @ state.diff(xinit0, MPC.x0)

        # Apply torque on complete model
        simulator.applyTorques(torques)
        # Compute one step of simulation
        simulator.step()

    MPC.iterate(pinWrapper.get_q0(), pinWrapper.get_v0(), pin.SE3.Identity())

simulator.end()

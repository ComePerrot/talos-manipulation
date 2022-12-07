#!/usr/bin/env python

#####################
#  LOADING MODULES  #
#####################

import pinocchio as pin
import numpy as np
import yaml

from sobec import RobotDesigner
from mpc_pointing import OCP_Point, OCPSettings_Point

from bullet_Talos import TalosDeburringSimulator
from plotter import TalosPlotter

from debug_ocp import (
    plot_costs_from_dic,
    return_cost_vectors,
    plot_command,
    return_command_vector,
    plot_state_from_dic,
    return_state_vector,
)

################
#  PARAMETERS  #
################

enableGUI = True
T_total = 500

modelPath = "/opt/openrobots/share/example-robot-data/robots/talos_data/"
URDF = modelPath + "robots/talos_reduced.urdf"
SRDF = modelPath + "srdf/talos.srdf"

# Parameters filename
filename = "../config/settings_sobec.yaml"

####################
#  INITIALIZATION  #
####################

# Loading extra parameters from file
with open(filename, "r") as paramFile:
    params = yaml.safe_load(paramFile)

controlledJoints = params["robot"]["controlledJoints"]
toolFramePos = params["robot"]["toolFramePos"]

OCPparams = OCPSettings_Point()

print("Loading data from file: \n" + filename)
OCPparams.readFromYaml(filename)
horizonLength = OCPparams.horizon_length


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
gripper_SE3_tool.translation[0] = toolFramePos[0]
gripper_SE3_tool.translation[1] = toolFramePos[1]
gripper_SE3_tool.translation[2] = toolFramePos[2]

pinWrapper.addEndEffectorFrame(
    "deburring_tool", "gripper_left_fingertip_3_link", gripper_SE3_tool
)

# OCP
oMtarget = pin.SE3.Identity()
oMtarget.translation[0] = 0.6
oMtarget.translation[1] = 0.4
oMtarget.translation[2] = 1.1

beta = -np.pi * 0.5
gamma = np.pi

rotationY = np.array(
    [[np.cos(beta), 0, -np.sin(beta)], [0, 1, 0], [np.sin(beta), 0, np.cos(beta)]]
)
rotationZ = np.array(
    [[np.cos(gamma), np.sin(gamma), 0], [-np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]]
)

oMtarget.rotation = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]])

print(oMtarget.rotation)

OCP = OCP_Point(OCPparams, pinWrapper)
OCP.initialize(pinWrapper.get_x0(), oMtarget)
print("OCP successfully loaded")

# Simulator
simulator = TalosDeburringSimulator(
    URDF=URDF,
    initialConfiguration=pinWrapper.get_q0Complete(),
    robotJointNames=pinWrapper.get_rModelComplete().names,
    controlledJointsIDs=pinWrapper.get_controlledJointsIDs(),
    toolPlacement=pinWrapper.get_EndEff_frame(),
    targetPlacement=oMtarget,
    enableGUI=enableGUI,
)

# Plotter
plotter = TalosPlotter(pinWrapper.get_rModel(), T_total)

###############
#  MAIN LOOP  #
###############

NcontrolKnots = 10
state = OCP.modelMaker.getState()
T = 0
toolPlacement = pinWrapper.get_EndEff_frame()

while T < T_total:
    # Controller works faster than trajectory generation
    for i_control in range(NcontrolKnots):
        if i_control == 0:
            x0 = simulator.getRobotState()

        x_measured = simulator.getRobotState()

        # Compute torque to be applied by adding Riccati term
        torques = OCP.torque + OCP.gain @ state.diff(x_measured, x0)

        # Apply torque on complete model
        simulator.step(torques, toolPlacement, oMtarget)

    # Solve MPC iteration
    pinWrapper.updateReducedModel(x_measured)
    OCP.solve(x_measured)

    # Update tool and target placement
    toolPlacement = pinWrapper.get_EndEff_frame()

    if T <= horizonLength:
        OCP.changeGoalCostActivation(horizonLength - T, True)

    # Log robot data
    plotter.logState(T, x_measured)
    plotter.logTorques(T, torques)
    plotter.logEndEffectorPos(T, toolPlacement.translation, oMtarget.translation)

    T += 1

simulator.end()
print("Simulation ended")

plotter.plotResults()

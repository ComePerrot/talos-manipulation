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

# Running cost
# state
# command
# DCM
# CoM
# Wrench
# Limit
# Position
# Orientation

# Terminal cost
# state
# DCM
# End-effector Velocity

enableGUI = True
plotResults = True
plotCosts = False

# Timing settings
T_total = 500
T_init = 50
T_GainScheduling = 10

gainScheduling = False
goalTrackingSlope = 0.5

targetTolerance = 0.007

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

oMtarget.rotation = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]])

OCP = OCP_Point(OCPparams, pinWrapper)
OCP.initialize(pinWrapper.get_x0(), oMtarget)
print("OCP successfully loaded")

ddp = OCP.solver

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

goalTrackingWeight = 10
drillingState = 0
targetReached = 0
reachTime = 0
ddp = OCP.solver

for T in range(T_total):
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

    if T < T_init:
        drillingState = 0
        pass
    elif T <= horizonLength + T_init:
        # Approach phase
        drillingState = 1
        OCP.changeGoalCostActivation(horizonLength + T_init - T, True)

        if T > horizonLength + T_init - T_GainScheduling:
            if gainScheduling:
                goalTrackingWeight += goalTrackingSlope
                OCP.changeGoalTrackingWeights(goalTrackingWeight)
        pass
    else:
        drillingState = 2
        pass

    # Log robot data
    plotter.logDrillingState(T, drillingState)
    plotter.logState(T, x_measured)
    plotter.logTorques(T, torques)
    plotter.logEndEffectorPos(T, toolPlacement.translation, oMtarget.translation)

    errorPlacement = np.linalg.norm(toolPlacement.translation - oMtarget.translation)

    if errorPlacement < targetTolerance:
        if not targetReached:
            targetReached = True
            reachTime = T - T_init
    else:
        targetReached = False

    # Check stop condition
    if toolPlacement.translation[1] > 1 or toolPlacement.translation[1] < 0:
        break

simulator.end()
print("Simulation ended")

# Reporting performances
print(errorPlacement / 1e-3)
print(reachTime / 100)

if plotResults:
    plotter.plotResults()

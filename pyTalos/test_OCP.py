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
plotResults = True
plotCosts = False

targetPos_1 = [0.6, 0.4, 1.1]
targetPos_2 = np.array([0.6, 0.4 + 0.1, 1.1])
targetTolerance = 0.005

# Timing settings
T_total = 300
T_init = 0

# Gain Scheduling
gainScheduling = False
T_GainScheduling = 10
goalTrackingSlope = 0.5

# Variable Posture
variablePosture = True
ref_leftArm = [
    -0.08419471,
    0.425144,
    0.00556666,
    -1.50516856,
    # 0.68574977,
    # 0.18184998,
    # -0.07185897,
]

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
oMtarget.translation[0] = targetPos_1[0]
oMtarget.translation[1] = targetPos_1[1]
oMtarget.translation[2] = targetPos_1[2]

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

########################
#  INITIAL RESOLUTION  #
########################

for index in range(horizonLength):
    OCP.changeGoalCostActivation(index, True)

x_measured = simulator.getRobotState()
OCP.solveFirst(pinWrapper.get_x0())

# plot_state_from_dic(return_state_vector(OCP.solver))

# ref_leftArm = OCP.solver.xs[-1][7 + 14 : 7 + 18]

for index in range(horizonLength):
    OCP.changeGoalCostActivation(index, False)
OCP.solveFirst(pinWrapper.get_x0())

###############
#  MAIN LOOP  #
###############
NcontrolKnots = 10
T = 0
drillingState = 0
targetReached = 0
reachTime = 0

state = OCP.modelMaker.getState()
toolPlacement = pinWrapper.get_EndEff_frame()
ddp = OCP.solver

goalTrackingWeight = 10

x_ref = pinWrapper.get_x0().copy()
x_ref[7 + 14 : 7 + 18] = ref_leftArm

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

    # Update tool placement
    toolPlacement = pinWrapper.get_EndEff_frame()

    # Handling phases of the movement
    if T < T_init:  # Initial phase
        drillingState = 0
        pass
    elif T <= horizonLength + T_init:  # Approach phase
        drillingState = 1
        index = horizonLength + T_init - T
        OCP.changeGoalCostActivation(index, True)

        if gainScheduling:
            if T > horizonLength + T_init - T_GainScheduling:
                if gainScheduling:
                    goalTrackingWeight += goalTrackingSlope
                    OCP.changeGoalTrackingWeights(goalTrackingWeight)

        if variablePosture:
            OCP.changePostureReference(index, x0)
        pass
    if T > horizonLength + T_init and T <= 2 * horizonLength + T_init:
        drillingState = 2
        index = 2 * horizonLength + T_init - T

        if index == 0:
            oMtarget.translation[0] = targetPos_2[0]
            oMtarget.translation[1] = targetPos_2[1]
            oMtarget.translation[2] = targetPos_2[2]

        OCP.changeTarget(index, targetPos_2)

    # else:
    #     drillingState = 3
    #     pass

    # Checking if target is reached
    errorPlacement = np.linalg.norm(toolPlacement.translation - oMtarget.translation)

    if errorPlacement < targetTolerance:
        if not targetReached:
            targetReached = True
            reachTime = T - T_init
    else:
        targetReached = False

    # Log robot data
    plotter.logDrillingState(T, drillingState)
    plotter.logState(T, x_measured)
    plotter.logTorques(T, torques)
    plotter.logEndEffectorPos(T, toolPlacement.translation, oMtarget.translation)
    plotter.logTargetReached(T, targetReached)

    pin.forwardKinematics(
        pinWrapper.get_rModel(),
        pinWrapper.get_rData(),
        x_measured[: pinWrapper.get_rModel().nq],
        x_measured[-pinWrapper.get_rModel().nv :],
    )
    speed = pin.getFrameVelocity(
        pinWrapper.get_rModel(),
        pinWrapper.get_rData(),
        pinWrapper.get_EndEff_id(),
    ).linear

    plotter.logEndEffectorSpeed(T, speed)

    # Check stop condition
    if toolPlacement.translation[1] > 1 or toolPlacement.translation[1] < 0:
        break

simulator.end
print("Simulation ended")

# Reporting performances
print("Target reached: " + str(targetReached))
print("Error=" + str(errorPlacement / 1e-3) + " mm")
print("Reach Time=" + str(reachTime / 100) + " s")

if plotResults:
    plotter.plotResults()

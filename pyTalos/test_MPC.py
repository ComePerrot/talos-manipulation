#!/usr/bin/env python

#####################
#  LOADING MODULES  #
#####################

import pinocchio as pin
import yaml

from deburring_mpc import MPC_Point, MPCSettings_Point, OCPSettings_Point, RobotDesigner

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
T_total = 3000

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

MPCparams = MPCSettings_Point()
OCPparams = OCPSettings_Point()

print("Loading data from file: \n" + filename)
OCPparams.readFromYaml(filename)
MPCparams.readFromYaml(filename)

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

# MPC
MPC = MPC_Point(MPCparams, OCPparams, pinWrapper)
MPC.initialize(
    pinWrapper.get_rModelComplete().referenceConfigurations["half_sitting"],
    pinWrapper.get_v0Complete(),
    pin.SE3.Identity(),
)

print("Robot successfully loaded")

ddp = MPC.OCP.solver

plot_state_from_dic(return_state_vector(ddp))
plot_command(return_command_vector(ddp))
plot_costs_from_dic(return_cost_vectors(MPC.OCP.solver, weighted=True))

# Simulator
simulator = TalosDeburringSimulator(
    URDF=URDF,
    initialConfiguration=pinWrapper.get_q0Complete(),
    robotJointNames=pinWrapper.get_rModelComplete().names,
    controlledJointsIDs=pinWrapper.get_controlledJointsIDs(),
    toolPlacement=pinWrapper.get_EndEff_frame(),
    targetPlacement=MPC.oMtarget,
    enableGUI=enableGUI,
)

# Plotter
plotter = TalosPlotter(pinWrapper.get_rModel(), T_total)

###############
#  MAIN LOOP  #
###############

NcontrolKnots = 10
state = MPC.OCP.state
T = 0
toolPlacement = MPC.designer.get_EndEff_frame()
targetPlacement = MPC.oMtarget

while T < T_total:
    # Controller works faster than trajectory generation
    for i_control in range(NcontrolKnots):
        x_measured = simulator.getRobotState()

        # Compute torque to be applied by adding Riccati term
        torques = MPC.u0 + MPC.K0 @ state.diff(x_measured, MPC.x0)

        # Apply torque on complete model
        simulator.step(torques, toolPlacement, targetPlacement)

    # Solve MPC iteration
    MPC.iterate(x_measured, pin.SE3.Identity())

    # Update tool and target placement
    toolPlacement = MPC.designer.get_EndEff_frame()
    targetPlacement = MPC.oMtarget

    # Log robot data
    plotter.logDrillingState(T, MPC.drillingState)
    plotter.logState(T, x_measured)
    plotter.logTorques(T, torques)
    plotter.logEndEffectorPos(T, toolPlacement.translation, targetPlacement.translation)

    T += 1

simulator.end()
print("Simulation ended")

plotter.plotResults()

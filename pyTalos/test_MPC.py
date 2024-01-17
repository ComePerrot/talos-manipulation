#!/usr/bin/env python

#####################
#  LOADING MODULES  #
#####################

import pinocchio as pin
import numpy as np
import yaml

from deburring_mpc import MPC, RobotDesigner

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
OCPparams = params["OCP"]

OCPparams["time_step"] = 0.01
OCPparams["state_weights"] = np.array(
    [
        500,
        500,
        500,
        1000,
        1000,
        1000,
        500,
        500,
        500,
        500,
        1000,
        1000,
        500,
        500,
        500,
        500,
        1000,
        1000,
        100,
        200,
        100,
        100,
        100,
        100,
        1,
        1,
        1,
        500,
        500,
        500,
        500,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        10,
        10,
        10,
        10,
        10,
        10,
        10,
        10,
        10,
        10,
        10,
        10,
        10,
    ]
)
OCPparams["control_weights"] = np.array(
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
)


MPCparams = params["MPC"]
MPCparams["target_position"] = np.array(MPCparams["target_position"])
MPCparams["holes_offsets"] = [np.array(offset) for offset in MPCparams["holes_offsets"]]
MPCparams["custom_arm_ref"] = np.array(MPCparams["custom_arm_ref"])

# Robot model
design_conf = dict(
    urdf_path=URDF,
    srdf_path=SRDF,
    left_foot_name="right_sole_link",
    right_foot_name="left_sole_link",
    robot_description="",
    controlled_joints_names=controlledJoints,
)
pinWrapper = RobotDesigner()
pinWrapper.initialize(design_conf)

gripper_SE3_tool = pin.SE3.Identity()
gripper_SE3_tool.translation[0] = toolFramePos[0]
gripper_SE3_tool.translation[1] = toolFramePos[1]
gripper_SE3_tool.translation[2] = toolFramePos[2]
pinWrapper.add_end_effector_frame(
    "deburring_tool", "gripper_left_fingertip_3_link", gripper_SE3_tool
)

# MPC
MPC = MPC(MPCparams, OCPparams, pinWrapper)
MPC.initialize(
    pinWrapper.get_rmodel_complete().referenceConfigurations["half_sitting"],
    pinWrapper.get_v0_complete(),
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
    initialConfiguration=pinWrapper.get_q0_complete(),
    robotJointNames=pinWrapper.get_rmodel_complete().names,
    controlledJointsIDs=pinWrapper.get_controlled_joints_ids(),
    toolPlacement=pinWrapper.get_end_effector_frame(),
    targetPlacement=MPC.oMtarget,
    enableGUI=enableGUI,
)

# Plotter
plotter = TalosPlotter(pinWrapper.get_rmodel(), T_total)

###############
#  MAIN LOOP  #
###############

NcontrolKnots = 10
state = MPC.OCP.state
T = 0
toolPlacement = MPC.designer.get_end_effector_frame()
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
    toolPlacement = MPC.designer.get_end_effector_frame()
    targetPlacement = MPC.oMtarget

    # Log robot data
    plotter.logDrillingState(T, MPC.drilling_state)
    plotter.logState(T, x_measured)
    plotter.logTorques(T, torques)
    plotter.logEndEffectorPos(T, toolPlacement.translation, targetPlacement.translation)

    T += 1

simulator.end()
print("Simulation ended")

plotter.plotResults()

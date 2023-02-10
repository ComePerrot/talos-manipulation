#!/usr/bin/env python

#####################
#  LOADING MODULES  #
#####################

import pinocchio as pin
import numpy as np
import yaml
import pickle

from sobec import RobotDesigner
from mpc_pointing import OCP_Point, OCPSettings_Point

################
#  PARAMETERS  #
################

targetPos = [0.6, 0.4, 1.1]

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

rModel = pinWrapper.get_rModel()

rModel.lowerPositionLimit = np.array(
    [
        # Base
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        # Left leg
        -0.35,
        -0.52,
        -2.10,
        0.0,
        -1.31,
        -0.52,
        # Right leg
        -1.57,
        -0.52,
        -2.10,
        0.0,
        -1.31,
        -0.52,
        # Torso
        -1.3,
        -0.1,
        # Left arm
        -1.57,
        -0.2,
        -2.44,
        -2.1,
        -2.53,
        -1.3,
        -0.6,
        # Right arm
        -0.4,
        -2.88,
        -2.44,
        -2.1,
    ]
)

rModel.upperPositionLimit = np.array(
    [
        # Base
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        # Left leg
        1.57,
        0.52,
        0.7,
        2.62,
        0.77,
        0.52,
        # Right leg
        0.35,
        0.52,
        0.7,
        2.62,
        0.77,
        0.52,
        # Torso
        1.3,
        0.78,
        # Left arm
        0.52,
        2.88,
        2.44,
        0,
        2.53,
        1.3,
        0.6,
        # Right arm
        1.57,
        -0.2,
        2.44,
        0,
    ]
)

# OCP
oMtarget = pin.SE3.Identity()
oMtarget.translation[0] = targetPos[0]
oMtarget.translation[1] = targetPos[1]
oMtarget.translation[2] = targetPos[2]

oMtarget.rotation = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]])

OCP = OCP_Point(OCPparams, pinWrapper)
OCP.initialize(pinWrapper.get_x0(), oMtarget)
print("OCP successfully loaded")

ddp = OCP.solver

########################
#  INITIAL RESOLUTION  #
########################

# Without target
with open("initialResolution_oldCroco.pkl", "rb") as file:
    # Call load method to deserialze
    x0 = pickle.load(file)
    xs_init_old = pickle.load(file)
    us_init_old = pickle.load(file)

OCP.solveFirst(x0)

xs_init_new = ddp.xs.tolist()
us_init_new = ddp.us.tolist()

for i in range(len(xs_init_old)):
    if (np.linalg.norm(xs_init_old[i] - xs_init_new[i])) > 1e-10:
        print("xs: " + str(i))
        print(np.linalg.norm(xs_init_old[i] - xs_init_new[i]))


for i in range(len(us_init_old)):
    if (np.linalg.norm(us_init_old[i] - us_init_new[i])) > 1e-10:
        print("us: " + str(i))
        print(np.linalg.norm(us_init_old[i] - us_init_new[i]))

print("Initial formulation, with no target, has been checked")

# With target
for index in range(horizonLength):
    OCP.changeGoalCostActivation(index, True)

with open("targetResolution_oldCroco.pkl", "rb") as file:

    # Call load method to deserialze
    x0 = pickle.load(file)
    xs_init_old = pickle.load(file)
    us_init_old = pickle.load(file)

OCP.solveFirst(x0)

xs_init_new = ddp.xs.tolist()
us_init_new = ddp.us.tolist()

for i in range(len(xs_init_old)):
    if (np.linalg.norm(xs_init_old[i] - xs_init_new[i])) > 1e-10:
        print("xs: " + str(i))
        print(np.linalg.norm(xs_init_old[i] - xs_init_new[i]))


for i in range(len(us_init_old)):
    if (np.linalg.norm(us_init_old[i] - us_init_new[i])) > 1e-10:
        print("us: " + str(i))
        print(np.linalg.norm(us_init_old[i] - us_init_new[i]))

print("Initial formulation, with target, has been checked")

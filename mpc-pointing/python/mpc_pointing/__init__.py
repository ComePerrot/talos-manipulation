# flake8: noqa

from .sobec_pywrap import (
    ResidualModelCoMVelocity,
    ResidualModelVelCollision,
    ActivationModelQuadRef,
    RobotDesigner,
    HorizonManager,
    ModelMaker,
    Support,
    ResidualModelCenterOfPressure,
    ResidualModelFeetCollision,
    ResidualModelFlyHigh,
    IntegratedActionModelLPF,
    StateLPF,
    ContactModel3D,
    ContactModel1D,
    ContactModelMultiple,
    DifferentialActionModelContactFwdDynamics,
    ResidualModelContactForce,
    WBC,
    OCPRobotWrapper,
    OCPWalkParams,
    OCPWalk,
    MPCWalkParams,
    MPCWalk,
    Flex,
    computeWeightShareSmoothProfile,
    LocomotionType,
    OCPSettings_Point,
    OCP_Point,
    MPCSettings_Point,
    MPC_Point
)

from .repr_ocp import reprProblem
from .viewer_multiple import GepettoGhostViewer
from . import logs

# TODO discuss with Guilhem if it is a good it to alias this.
from . import walk_without_think as wwt
from . import walk_without_think

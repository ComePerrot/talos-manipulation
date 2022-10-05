import example_robot_data
import numpy as np
import pinocchio as pin
import pybullet as p  # PyBullet simulator
import pybullet_data
import queue


def a2m(a):
    return np.matrix(a).T


def m2a(m):
    return np.array(m.flat)


class TalosDeburringSimulator:
    def __init__(
        self,
        URDF_File,
        URDF_Path,
        targetPos,
        rmodel,
        ControlledJoints,
        initialConfiguration,
        enableGUI=False,
        enableGravity=True,
        dt=1e-3,
    ):

        # Start the client for PyBullet
        self.enableGUI = enableGUI
        if self.enableGUI:
            self.physicsClient = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.physicsClient = p.connect(p.DIRECT)

        # Set gravity (enabled by default)
        self.enableGravity = enableGravity
        if enableGravity:
            p.setGravity(0, 0, -9.81)
        else:
            p.setGravity(0, 0, 0)

        # Set time step of the simulation
        self.dt = dt
        p.setTimeStep(self.dt)

        # Load horizontal plane for PyBullet
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # Load the robot for PyBullet
        modelPath = example_robot_data.getModelPath(URDF_Path)
        p.setAdditionalSearchPath(modelPath + URDF_Path)

        robotStartPosition = [0.0, 0.0, 1.01927]
        robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotId = p.loadURDF(
            URDF_File,
            robotStartPosition,
            robotStartOrientation,
            useFixedBase=False,
        )

        self.createTargetVisual(targetPos)
        self.setControlledJoints(rmodel, ControlledJoints)
        self.setInitialConfig(initialConfiguration)
        self.createToolVisual()

        # Define usefull attributes for the simulation
        self.localInertiaPos = np.matrix(
            p.getDynamicsInfo(self.robotId, -1)[3]
        ).T

        self.bulletFrameId = 20  # 23-left fingertip 20-gripper_left_joint

        # Variables to simulate delay in the measures
        self.valueDelay = 10
        self.queueDelay = queue.Queue()

    def setInitialConfig(self, q0):
        """Initialize robot configuration in pyBullet

        :param q0 Intial robot configuration
        """
        initial_joint_positions = np.array(q0[7:].flat).tolist()
        for i in range(len(initial_joint_positions)):
            p.enableJointForceTorqueSensor(1, i, True)
            p.resetJointState(
                self.robotId,
                self.JointIndicesComplete[i],
                initial_joint_positions[i],
            )

    def setControlledJoints(self, rmodelComplete, ControlledJoints):
        """Define joints controlled by pyBullet

        :param rmodelComplete Complete model of the robot
        :param ControlledJoints List of ControlledJoints
        """
        bulletJointNames = [
            p.getJointInfo(self.robotId, i)[1].decode()
            for i in range(p.getNumJoints(self.robotId))
        ]
        self.JointIndicesComplete = [
            bulletJointNames.index(rmodelComplete.names[i])
            for i in range(2, rmodelComplete.njoints)
        ]

        # Joints controlled with crocoddyl
        self.bulletControlledJoints = [
            i
            for i in self.JointIndicesComplete
            if p.getJointInfo(self.robotId, i)[1].decode()
            in ControlledJoints
        ]

        # Disable default position controler in torque controlled joints
        # Default controller will take care of other joints
        p.setJointMotorControlArray(
            self.robotId,
            jointIndices=self.bulletControlledJoints,
            controlMode=p.VELOCITY_CONTROL,
            forces=[0.0 for m in self.bulletControlledJoints],
        )

        # Augment friction to forbid feet sliding
        p.changeDynamics(1, 50, lateralFriction=100, spinningFriction=30)
        p.changeDynamics(1, 57, lateralFriction=100, spinningFriction=30)

    def createTargetVisual(self, target):
        """Create visual representation of the target to track

        The visual will not appear unless the physics client is set to
        SHARED_MEMMORY
        :param target Position of the target in the world
        """
        RADIUS = 0.01
        LENGTH = 0.02
        blueBox = p.createVisualShape(
            shapeType=p.GEOM_CAPSULE,
            rgbaColor=[0, 0, 1, 1.0],
            visualFramePosition=[0.0, 0.0, 0.0],
            radius=RADIUS,
            length=LENGTH,
            halfExtents=[0.0, 0.0, 0.0],
        )

        self.target_MPC = p.createMultiBody(
            baseMass=0.0,
            baseInertialFramePosition=[0, 0, 0],
            baseVisualShapeIndex=blueBox,
            basePosition=[target[0], target[1], target[2]],
            useMaximalCoordinates=True,
        )

        greenBox = p.createVisualShape(
            shapeType=p.GEOM_CAPSULE,
            rgbaColor=[0, 1, 0, 1.0],
            visualFramePosition=[0.0, 0.0, 0.0],
            radius=RADIUS,
            length=LENGTH,
            halfExtents=[0.0, 0.0, 0.0],
        )

        self.target_bullet = p.createMultiBody(
            baseMass=0.0,
            baseInertialFramePosition=[0, 0, 0],
            baseVisualShapeIndex=greenBox,
            basePosition=[target[0], target[1], target[2]],
            baseOrientation=[0, -0.707107, 0, 0.707107],
            useMaximalCoordinates=True,
        )

    def createToolVisual(self):
        """Create visual representation of the robot end effector"""
        RADIUS = 0.01
        LENGTH = 0.02
        blueCapsule = p.createVisualShape(
            shapeType=p.GEOM_CAPSULE,
            rgbaColor=[0, 0, 1, 0.5],
            visualFramePosition=[0.0, 0.0, 0.0],
            radius=RADIUS,
            length=LENGTH,
            halfExtents=[0.0, 0.0, 0.0],
        )
        self.tool_pin = p.createMultiBody(
            baseMass=0.0,
            baseInertialFramePosition=[0, 0, 0],
            baseVisualShapeIndex=blueCapsule,
            basePosition=[0.0, 0.0, 0.0],
            useMaximalCoordinates=True,
        )
        greenCapsule = p.createVisualShape(
            shapeType=p.GEOM_CAPSULE,
            rgbaColor=[0, 1, 0, 0.5],
            visualFramePosition=[0.0, 0.0, 0.0],
            radius=RADIUS,
            length=LENGTH,
            halfExtents=[0.0, 0.0, 0.0],
        )
        self.tool_bullet = p.createMultiBody(
            baseMass=0.0,
            baseInertialFramePosition=[0, 0, 0],
            baseVisualShapeIndex=greenCapsule,
            basePosition=[0.0, 0.0, 0.0],
            useMaximalCoordinates=True,
        )

    def setToolPosition(self, placementTool_pin):
        """Move the robot's capsule according to current robot's position"""
        # @TODO Cleanup the conversion between different types of rotations

        placementTool_bullet = placementTool_pin

        toolTranslation_pin = m2a(placementTool_pin.translation)
        toolQuaternion_pin = pin.Quaternion(placementTool_pin.rotation)
        p.resetBasePositionAndOrientation(
            self.tool_pin,
            toolTranslation_pin,
            np.array(
                [
                    toolQuaternion_pin.x,
                    toolQuaternion_pin.y,
                    toolQuaternion_pin.z,
                    toolQuaternion_pin.w,
                ]
            ),
        )

        toolTranslation_bullet = m2a(placementTool_bullet.translation)
        toolQuaternion_bullet = pin.Quaternion(placementTool_bullet.rotation)
        p.resetBasePositionAndOrientation(
            self.tool_bullet,
            toolTranslation_bullet,
            np.array(
                [
                    toolQuaternion_bullet.x,
                    toolQuaternion_bullet.y,
                    toolQuaternion_bullet.z,
                    toolQuaternion_bullet.w,
                ]
            ),
        )

    def getToolMTarget(self, oMtool_pin):
        self.setToolPosition(oMtool_pin)

        # target state as seen by the simulator (represents MOCAP measure)
        posTarget, rotTarget = p.getBasePositionAndOrientation(
            self.target_bullet
        )
        oMtarget = pin.SE3(
            pin.Quaternion(a2m(np.array(rotTarget))), a2m(np.array(posTarget))
        )

        # tool state as seen by the simulator (represents MOCAP measure)
        posTool, rotTool = p.getBasePositionAndOrientation(self.tool_bullet)
        oMtool = pin.SE3(
            pin.Quaternion(a2m(np.array(rotTool))), a2m(np.array(posTool))
        )

        toolMtarget = oMtool.actInv(oMtarget)

        # add Gaussian noise to the measure
        translationNoise = np.random.normal(0, 0.0003, 3)
        toolMtarget.translation += np.matrix(
            [
                [translationNoise[0]],
                [translationNoise[1]],
                [translationNoise[2]],
            ]
        )

        if self.queueDelay.empty():
            for i in range(self.valueDelay):
                self.queueDelay.put(toolMtarget)
        else:
            self.queueDelay.put(toolMtarget)

        return self.queueDelay.get()

    def getCurrenRobotState(self):
        """Get current state of the robot from pyBullet"""
        jointStates = p.getJointStates(
            self.robotId, self.JointIndicesComplete
        )  # State of all joints
        baseState = p.getBasePositionAndOrientation(self.robotId)
        baseVel = p.getBaseVelocity(self.robotId)

        # Joint vector for Pinocchio
        q = np.vstack(
            (
                np.array([baseState[0]]).transpose(),
                np.array([baseState[1]]).transpose(),
                np.array(
                    [
                        [
                            jointStates[i_joint][0]
                            for i_joint in range(len(jointStates))
                        ]
                    ]
                ).transpose(),
            )
        )
        v = np.vstack(
            (
                np.array([baseVel[0]]).transpose(),
                np.array([baseVel[1]]).transpose(),
                np.array(
                    [
                        [
                            jointStates[i_joint][1]
                            for i_joint in range(len(jointStates))
                        ]
                    ]
                ).transpose(),
            )
        )

        return (q, v)

    def applyTorques(self, torques):
        """Apply computed torques to the robot"""
        p.setJointMotorControlArray(
            self.robotId,
            self.bulletControlledJoints,
            controlMode=p.TORQUE_CONTROL,
            forces=torques,
        )

    def step(self):
        """Do one step of simulation"""
        p.stepSimulation()

    def setTargetPosition(self, reference):
        translation = m2a(reference.translation)
        rotation = pin.Quaternion(reference.rotation)

        p.resetBasePositionAndOrientation(
            self.target_MPC,
            posObj=translation,
            ornObj=np.array([rotation.x, rotation.y, rotation.z, rotation.w]),
        )

    def translateTarget(self, translation):
        posTarget, rotTarget = p.getBasePositionAndOrientation(
            self.target_bullet
        )
        x, y, z = posTarget

        p.resetBasePositionAndOrientation(
            self.target_bullet,
            posObj=(x + translation, y, z),
            ornObj=rotTarget,
        )

    def end(self):
        """Ends connection with pybullet."""
        p.disconnect()

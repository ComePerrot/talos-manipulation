import matplotlib.pylab as plt
import numpy as np


class TalosPlotter:
    def __init__(self, rmodel, T_total):
        self.rmodel = rmodel

        self.drillingState = np.zeros(T_total)
        self.targetReached = np.zeros(T_total)
        self.x = np.zeros((T_total, 63))
        self.u = np.zeros((T_total, 25))
        self.actualPos = np.zeros((T_total, 3))
        self.desiredPos = np.zeros((T_total, 3))
        self.EndEffectorSpeed = np.zeros((T_total, 3))

    def logDrillingState(self, T_current, drillingState):
        self.drillingState[T_current] = drillingState

    def logTargetReached(self, T_current, targetReached):
        self.targetReached[T_current] = targetReached

    def logState(self, T_current, x_current):
        self.x[T_current] = x_current.flat

    def logTorques(self, T_current, torques):
        self.u[T_current] = torques.flat

    def logEndEffectorPos(self, T_current, acutal_pos, desired_pos):
        self.actualPos[T_current] = acutal_pos.flat
        self.desiredPos[T_current] = desired_pos.flat

    def logEndEffectorSpeed(self, T_current, speed):
        self.EndEffectorSpeed[T_current] = speed.flat

    def plotResults(self):
        plt.figure()
        plt.subplot(411)
        plt.title("Distance in x")
        plt.plot(self.actualPos[:, 0], label="Actual x")
        plt.plot(self.desiredPos[:, 0], label="Desired x")
        plt.legend(loc="lower right")
        plt.grid(True)
        plt.subplot(412)
        plt.title("Distance in y")
        plt.plot(self.actualPos[:, 1], label="Actual y")
        plt.plot(self.desiredPos[:, 1], label="Desired y")
        plt.legend(loc="lower right")
        plt.grid(True)
        plt.subplot(413)
        plt.title("Distance in z")
        plt.plot(self.actualPos[:, 2], label="Actual z")
        plt.plot(self.desiredPos[:, 2], label="Desired z")
        plt.legend(loc="lower right")
        plt.grid(True)
        plt.subplot(414)
        plt.title("Drilling state")
        plt.plot(self.drillingState, label="Drilling state")
        plt.plot(self.targetReached, label="target reached")
        plt.legend(loc="lower right")
        plt.xlabel("Time")
        plt.grid(True)

        plt.figure()
        plt.subplot(311)
        plt.title("Velocity in x")
        plt.plot(np.abs(self.EndEffectorSpeed[:, 0]))
        plt.subplot(312)
        plt.title("Velocity in y")
        plt.plot(np.abs(self.EndEffectorSpeed[:, 1]))
        plt.subplot(313)
        plt.title("Velocity in z")
        plt.plot(np.abs(self.EndEffectorSpeed[:, 2]))

        plt.show()

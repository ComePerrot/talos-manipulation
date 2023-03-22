# Deburring Model Predictive Controller

Implementation of a model predictive controller based on Crocoddyl to carry out deburring tasks with the TALOS robot.

## Structure
### Robot Designer

Class that handles the robot model.
It is a wrapper around pinocchio.

### OCP (Optimal Control Problem)

Class that handles the generation and the resolution of the Optimal Control Problem.
It is a wrapper around Crocoddyl.

### MPC

Class that implements the general logic to carry out a task.
For now it contains a simple implementation of a finite state machine.

## Dependencies

* [Boost](https://www.boost.org/)
* [Crocoddyl](https://github.com/loco-3d/crocoddyl)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [eigenpy](https://github.com/stack-of-tasks/eigenpy)
* [pinocchio](https://github.com/stack-of-tasks/pinocchio)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp)
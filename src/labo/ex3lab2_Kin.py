import pyro

# Generic python tools
import numpy as np
from IPython import display

from pyro.dynamic import manipulator

# Dynamic model (inputs are motor torques)
torque_controlled_robot    = manipulator.FiveLinkPlanarManipulator()

# Kinematic only model (inputs are motor velocities)
speed_controlled_robot  = manipulator.SpeedControlledManipulator.from_manipulator( torque_controlled_robot )

robot = speed_controlled_robot # For this exercise, we will only use the kinematic model

# robot initial states [ joint 1 angle (rad),  joint 2 angle (rad), joint 1 velocity (rad/sec),  joint 1 velocity (rad/sec)]
robot.x0 = np.zeros(5)

# robot constant inputs
robot.ubar = np.array([ 0.5, 0.5, 0.5, 0.5, 0.5]) # Constant joint velocities

# run the simulation
robot.compute_trajectory( tf = 6 )

# Animate and display the simulation
#robot.animate_simulation()

from pyro.control import robotcontrollers

# Controller
ctl       = robotcontrollers.EndEffectorKinematicController( robot )
ctl.rbar  = np.array([1.0,1.0]) # target effector position
ctl.gains = np.array([2.0,2.0]) # gains

# Closed-loop dynamics
cl_sys  = ctl + robot

# Initial config
cl_sys.x0 = np.array([0.1,0.1,0.1,0.1,0.1])

# Simulation
cl_sys.compute_trajectory( tf = 5 )

cl_sys.animate_simulation()


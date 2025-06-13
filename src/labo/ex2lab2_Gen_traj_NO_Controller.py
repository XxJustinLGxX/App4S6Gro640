import pyro

# Generic python tools
import numpy as np
from IPython import display

from pyro.dynamic import manipulator

# Dynamic model (inputs are motor torques)
torque_controlled_robot    = manipulator.TwoLinkManipulator()

torque_controlled_robot.l1 = 4.0 # length of first rigid link
torque_controlled_robot.l2 = 3.0 # length of second rigid link
torque_controlled_robot.l_domain = 10

# Kinematic only model (inputs are motor velocities)
speed_controlled_robot  = manipulator.SpeedControlledManipulator.from_manipulator( torque_controlled_robot )

robot = speed_controlled_robot # For this exercise, we will only use the kinematic model

# robot initial states [ joint 1 angle (rad),  joint 2 angle (rad), joint 1 velocity (rad/sec),  joint 1 velocity (rad/sec)]
robot.x0 = np.array([ 0.1, 0.1])

# robot constant inputs
robot.ubar = np.array([ 0.5, 1.0]) # Constant joint velocities

# run the simulation
robot.compute_trajectory( tf = 6 )

# Animate and display the simulation
robot.animate_simulation()




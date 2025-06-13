import pyro

# Generic python tools
import numpy as np
from IPython import display

from pyro.dynamic import manipulator

robot = manipulator.TwoLinkManipulator()

robot.l1 = 0.5 # length of first rigid link
robot.l2 = 0.3 # length of second rigid link

# robot initial states [ joint 1 angle (rad),  joint 2 angle (rad), joint 1 velocity (rad/sec),  joint 1 velocity (rad/sec)]
robot.x0 = np.array([ 0.1, 0.1, 0.0, 0.0])

# run the simulation
robot.compute_trajectory( tf = 6 )

# Animate and display the simulation
robot.animate_simulation()
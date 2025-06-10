# Loading a robotic toolbox
# import sys
# sys.path.append('/content/pyro')
import pyro
import numpy as np
from IPython import display

from pyro.dynamic import manipulator

robot = manipulator.TwoLinkManipulator()

robot.l1 = 4.0 # length of first rigid link
robot.l2 = 3.0 # length of second rigid link
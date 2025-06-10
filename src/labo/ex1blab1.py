# Loading a robotic toolbox
# import sys
# sys.path.append('/content/pyro')
import pyro
import numpy as np
from IPython import display

from pyro.dynamic import manipulator

# Dynamic model (inputs are motor torques)
torque_controlled_robot    = manipulator.TwoLinkManipulator()

torque_controlled_robot.l1 = 3.0 # length of first rigid link
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
#ani  = robot.generate_simulation_html_video()
#html = display.HTML( ani )
#display.display(html)

from pyro.control  import robotcontrollers

class CustomKinematicController( robotcontrollers.EndEffectorKinematicController ) :
    def __init__(self, robot, k=1):
        super().__init__(robot, k)
        self.kp = 2

    #############################
    def c( self , y , r , t ):
        """
        Feedback static computation u = c(y,r,t)

        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1

        OUTPUTS
        dq  : joint velocity vector   m x 1

        """

        # Feedback from sensors
        q = y                  # Joint angles

        # Pre-computed values based on the robot kinematic model
        J = self.J( q )        # Jacobian computation
        r = self.fwd_kin( q )  # End-effector postion

        ##############################
        # YOUR CODE BELLOW !!
        ##############################

        # Compute the reference
        #r_d  = np.zeros(2) # Place-holder
        #dr_d = np.zeros(2) # Place-holder

        # Compute the desired effector velocity
        #dr_r = np.array([-0.1,0.1]) # Place holder
        theta = t**2*0.5
        r_d = np.array([1.5*np.cos(theta), 1.5*np.sin(theta)])
        dr_d = np.array([-1.5*np.sin(theta)*t, 1.5*np.cos(theta)*t])
        erreur = r_d - r
        dr_r = self.kp * erreur + dr_d
        # From effector speed to joint speed
        dq = np.linalg.inv( J ) @ dr_r

        return dq

ctl = CustomKinematicController(robot, 1)

# Create the closed-loop system
robot_with_controller = ctl + robot

# Run the simulation
robot_with_controller.x0[0] = 1.0
robot_with_controller.x0[1] = 1.0
robot_with_controller.compute_trajectory( tf = 6 )

# Plot systems states
robot_with_controller.plot_trajectory('x')

# Plot control inputs
robot_with_controller.plot_trajectory('u')

# Animate and display the simulation
robot_with_controller.animate_simulation()
#ani  = robot.generate_simulation_html_video()
#html = display.HTML( ani )
#display.display(html)
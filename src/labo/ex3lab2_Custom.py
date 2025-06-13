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

from pyro.control  import robotcontrollers

class CustomKinematicController( robotcontrollers.EndEffectorKinematicController ) :

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
        r      = self.fwd_kin( q )    # End-effector postion
        J      = self.J( q )          # Jacobian computation
        J_pinv = np.linalg.pinv( J )  # Pseudo Inverse
        Null_p = np.identity( self.dof ) - np.dot(J_pinv,J) # Nullspace projection Matrix

        ##############################
        # YOUR CODE BELLOW !!
        ##############################

        dq = Null_p @ np.array([1.0,0,0,0,0])

        return dq
    
ctl = CustomKinematicController( robot )

# Create the closed-loop system
robot_with_controller = ctl + robot

# Run the simulation
robot_with_controller.x0 = np.array([0.1,0.2,0.2,0.2,0.2])
robot_with_controller.compute_trajectory( tf = 3 )

robot_with_controller.animate_simulation()

# Plot systems states
robot_with_controller.plot_trajectory('x')

# Plot control inputs
robot_with_controller.plot_trajectory('u')
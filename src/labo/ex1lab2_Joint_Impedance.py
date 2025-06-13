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
#robot.animate_simulation()

from pyro.control  import robotcontrollers

# Target
q_desired = np.array([0.5,0.5]) # this set the desired joint angles vector

# Joint PID
joint_pd      = robotcontrollers.JointPD( dof = 2 )
joint_pd.rbar = q_desired
joint_pd.kp   = np.array([5, 5 ]) # This set the diagonal values of the Kp matrix
joint_pd.kd   = np.array([ 0, 0 ]) # This set the diagonal values of the Kd matrix

# This shows the commanded torque as a function of the configuration (at zero velocity)
#joint_pd.plot_control_law( sys = robot , n = 100 )

# Create the closed-loop system
robot_with_joint_pd = joint_pd + robot

# robot initial states [ joint 1 angle (rad),  joint 2 angle (rad), joint 1 velocity (rad/sec),  joint 1 velocity (rad/sec)]
#closed_loop_robot.x0 = np.array([0.1,0.1,0,0])

# Run the simulation
robot_with_joint_pd.compute_trajectory( tf = 5 )

# Animate the simulation
robot_with_joint_pd.animate_simulation()

# Plot systems states
robot_with_joint_pd.plot_trajectory('x')

# Plot control inputs
robot_with_joint_pd.plot_trajectory('u')

robot_with_joint_pd.plot_linearized_bode()

# End-effector desired position
r_desired = robot.forward_kinematic_effector( q_desired )
print('Target joint angles [q_1,q_2] =', q_desired )
print('Target effector [x,y] =', r_desired )

effector_pd      = robotcontrollers.EndEffectorPD( robot )
effector_pd.rbar = r_desired
effector_pd.kp   = np.array([100, 100 ]) # This set the diagonal values of the Kp matrix
effector_pd.kd   = np.array([  0,   0 ]) # This set the diagonal values of the Kd matrix

# This shows graphically the computed torque for joint 1 as a function of the configuration
effector_pd.plot_control_law( sys = robot , n = 100 )

# Create the closed-loop system
robot_with_effector_pd = effector_pd + robot

# robot initial states [ joint 1 angle (rad),  joint 2 angle (rad), joint 1 velocity (rad/sec),  joint 1 velocity (rad/sec) ]
#closed_loop_robot.x0 = np.array([0.1,0.1,0,0,0,0])

# Run the simulation
robot_with_effector_pd.compute_trajectory( tf = 5 )

robot_with_effector_pd.animate_simulation()

robot_with_effector_pd.plot_trajectory('x')

robot_with_effector_pd.plot_trajectory('u')

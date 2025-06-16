#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

from pyro.control import robotcontrollers
from pyro.control.robotcontrollers import EndEffectorPD
from pyro.control.robotcontrollers import EndEffectorKinematicController


###################
# Part 1
###################


def dh2T(r, d, theta, alpha):
    """

    Parameters
    ----------
    r     : float 1x1
    d     : float 1x1
    theta : float 1x1
    alpha : float 1x1

    4 paramètres de DH

    Returns
    -------
    T     : float 4x4 (numpy array)
            Matrice de transformation

    """

    T = np.zeros((4, 4), dtype=np.float64)
    
    ###################
    # Votre code ici
    ###################
    #colonne 1
    T[0][0] = np.cos(theta)
    T[1][0] = np.sin(theta)
    T[2][0] = 0.0
    T[3][0] = 0.0
    #colonne 2
    T[0][1] = -np.sin(theta)*np.cos(alpha)
    T[1][1] = np.cos(theta)*np.cos(alpha)
    T[2][1] = np.sin(alpha)
    T[3][1] = 0.0
    #colonne 3
    T[0][2] = np.sin(theta)*np.sin(alpha)
    T[1][2] = -np.cos(theta)*np.sin(alpha)
    T[2][2] = np.cos(alpha)
    T[3][2] = 0.0
    #colonne 4
    T[0][3] = r*np.cos(theta)
    T[1][3] = r*np.sin(theta)
    T[2][3] = d
    T[3][3] = 1.0
    print(T)
    print("1on1")
    return T


def dhs2T(r, d, theta, alpha):
    """

    Parameters
    ----------
    r     : float nx1
    d     : float nx1
    theta : float nx1
    alpha : float nx1

    Colonnes de paramètre de DH

    Returns
    -------
    WTT     : float 4x4 (numpy array)
              Matrice de transformation totale de l'outil

    """
    
    
    WTT = np.zeros((4, 4), dtype=np.float64)


    ###################
    # Votre code ici
    ###################
    for i in range(6):
        if i == 0:
            WTT = dh2T(r[i], d[i], theta[i], alpha[i])
        else:
            WTT = WTT @ dh2T(r[i], d[i], theta[i], alpha[i])
        print(WTT)
        print(i)

    return WTT


def f(q):
    """


    Parameters
    ----------
    q : float 6x1
        Joint space coordinates

    Returns
    -------
    r : float 3x1
        Effector (x,y,z) position

    """
    r_out = np.zeros((3, 1))

    r = np.array([-0.033, 0.155, 0.118, 0.0, 0.0, 0.006], dtype=np.float64)
    d = np.array([-0.147, 0.0, 0.019, -0.009, -0.217, 0.009+q[5]], dtype=np.float64)
    theta = np.array([q[0]+(np.pi*0.5), q[1]+(np.pi*0.5), q[2], q[3]-(np.pi*0.5),  q[4], 0.0],dtype=np.float64)
    alpha = np.array([-np.pi*0.5, 0.0, 0.0, np.pi*0.5, -np.pi*0.5, -np.pi*0.5],dtype=np.float64)
    
    ###################
    # Votre code ici
    ###################
    T = dhs2T(r, d, theta, alpha)

    r_out[0] = T[0][3]
    r_out[1] = T[1][3]
    r_out[2] = T[2][3]

    return r_out


###################
# Part 2
###################


class CustomPositionController(EndEffectorKinematicController):
    ############################
    def __init__(self, manipulator):
        """ """

        EndEffectorKinematicController.__init__(self, manipulator, 1)

        ###################################################
        # Vos paramètres de loi de commande ici !!
        ###################################################

    #############################
    def c(self, y, r, t=0):
        """
        Feedback law: u = c(y,r,t)

        INPUTS
        y = q   : sensor signal vector  = joint angular positions      dof x 1
        r = r_d : reference signal vector  = desired effector position   e x 1
        t       : time                                                   1 x 1

        OUPUTS
        u = dq  : control inputs vector =  joint velocities             dof x 1

        """

        # Feedback from sensors
        q = y

        # Jacobian computation
        J = self.J(q)

        # Ref
        r_desired = r
        r_actual = self.fwd_kin(q)

        # Error
        e = r_desired - r_actual

        ################
        dq = np.zeros(self.m)  # place-holder de bonne dimension

        ##################################
        # Votre loi de commande ici !!!
        ##################################
        lmd = 0.3
    
        dq = np.linalg.pinv(np.transpose(J)@J+lmd**2*np.identity(len(y)))@np.transpose(J)@e

        return dq


###################
# Part 3
###################


class CustomDrillingController(robotcontrollers.RobotController):
    """ """

    ############################
    def __init__(self, robot_model):
        """ """

        super().__init__(dof=3)

        self.robot_model = robot_model

        # Label
        self.name = "Custom Drilling Controller"

    #############################
    def c(self, y, r, t=0):
        """
        Feedback static computation u = c(y,r,t)

        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1

        OUPUTS
        u  : control inputs vector    m x 1

        """
        # Ref
        f_e = r

        # Feedback from sensors
        x = y
        [q, dq] = self.x2q(x)

        # Robot model
        r = self.robot_model.forward_kinematic_effector(q)  # End-effector actual position
        J = self.robot_model.J(q)  # Jacobian matrix
        g = self.robot_model.g(q)  # Gravity vector
        H = self.robot_model.H(q)  # Inertia matrix
        C = self.robot_model.C(q, dq)  # Coriolis matrix

        ##################################
        # Votre loi de commande ici !!!
        #################################
        
        Kx = 50 # constante de ressort
        fe = np.zeros((2,1))
        fmax = np.zeros((2,1))
        K = np.zeros((2,2))
        K[0][0] = Kx
        fmax[1] = -200
        #e = 
        #fe = K@
        
        u = np.zeros(self.m)  # place-holder de bonne dimension

        #u = np.transpose(J)@fe + g

        return u


###################
# Part 4
###################


def goal2r(r_0, r_f, t_f):
    """

    Parameters
    ----------
    r_0 : numpy array float 3 x 1
        effector initial position
    r_f : numpy array float 3 x 1
        effector final position
    t_f : float
        time

    Returns
    -------
    r   : numpy array float 3 x l
    dr  : numpy array float 3 x l
    ddr : numpy array float 3 x l

    """
    # Time discretization
    l = 1000  # nb of time steps

    # Number of DoF for the effector only
    m = 3

    r = np.zeros((m, l))
    dr = np.zeros((m, l))
    ddr = np.zeros((m, l))

    #################################
    # Votre code ici !!!
    ##################################

    return r, dr, ddr


def r2q(r, dr, ddr, manipulator):
    """

    Parameters
    ----------
    r   : numpy array float 3 x l
    dr  : numpy array float 3 x l
    ddr : numpy array float 3 x l

    manipulator : pyro object

    Returns
    -------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l

    """
    # Time discretization
    l = r.shape[1]

    # Number of DoF
    n = 3

    # Output dimensions
    q = np.zeros((n, l))
    dq = np.zeros((n, l))
    ddq = np.zeros((n, l))

    #################################
    # Votre code ici !!!
    ##################################

    return q, dq, ddq


def q2torque(q, dq, ddq, manipulator):
    """

    Parameters
    ----------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l

    manipulator : pyro object

    Returns
    -------
    tau   : numpy array float 3 x l

    """
    # Time discretization
    l = q.shape[1]

    # Number of DoF
    n = 3

    # Output dimensions
    tau = np.zeros((n, l))

    #################################
    # Votre code ici !!!
    ##################################

    return tau

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
        self.case = 1

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
        
        Fe = np.array([0, 0, -200])
        
        Kp = np.array([[100.0, 0.0, 0.0],[0.0, 100.0, 0.0],[0.0, 0.0, 100.0]])
        Kd = np.array([[25.0, 0.0, 0.0],[0.0, 25.0, 0.0],[0.0, 0.0, 25.0]])
        
        u = np.zeros(self.m)  # place-holder de bonne dimension
        Jtranp = np.transpose(J)

        if(self.case == 1) :
            r_g = np.array([0.25, 0.25, 0.5]) # bouger robot au dessu du preperçage 
            e = r_g - r
            u = Jtranp@(Kp@e + Kd@(-J@dq)) + g
            # robot pret pour descendre pour placer meche dans preperçage
            if(np.linalg.norm(e) < 0.01 ): #10mm d'erreur
                self.case = 2

        if(self.case == 2) :
            r_g = np.array([0.25, 0.25, 0.4]) # bouger robot au preperçage 
            e = r_g - r
            u = Jtranp@(Kp@e + Kd@(-J@dq)) + g
            # robot pret a percer
            if(np.linalg.norm(e) < 0.001 ): #1mm d'erreur
                self.case = 3

        #if(self.case == 3) :
        #    r_g = np.array([0.25, 0.25, 0.2]) # perçage
        #    e = r_g - r
        #    u = Jtranp @ Fe + g
        #    # robot fini perçage
        #    if(r[2] < 0.2 ): #Fini de percer
        #        self.case = 4
        
        if(self.case == 3) :
            r_g = np.array([0.25, 0.25, 0.2]) # perçage
            e = r_g - r
            Kp = np.array([[80.0, 0.0, 0.0],[0.0, 80.0, 0.0],[0.0, 0.0, 1.0]])
            Kd = np.array([[25.0, 0.0, 0.0],[0.0, 25.0, 0.0],[0.0, 0.0, 1.0]])
            u = Jtranp @Fe + g + Jtranp@(Kp@e + Kd@(-J@dq)) #
            # robot fini perçage
            if(r[2] < 0.2 ): #Fini de percer
                self.case = 4

        if(self.case == 4) :
            r_g = np.array([0.25, 0.25, 0.2]) # perçage
            e = r_g - r
            Kp = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
            Kd = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
            u = np.transpose(J)@Fe + g + np.transpose(J)@(Kp@e + Kd@(-J@dq))
            # robot fini perçage

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

    t = np.linspace(0, t_f, l)  # vecteur temps

    # Chemin géométrique (ligne droite)
    delta_r = r_f - r_0

    # Profil temporel s(t), ds(t), dds(t) 
    s = 3 * (t / t_f) ** 2 - 2 * (t / t_f) ** 3         
    ds = 6* (t/ t_f**2) - 6* ( t**2 / t_f**3)         
    dds = (6 / t_f**2) - (12*t / t_f**3)

    for i in range(l):
        r[:, i]   = r_0 + s[i]   * delta_r            
        dr[:, i]  = ds[i]  * delta_r                 
        ddr[:, i] = dds[i] * delta_r                  

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

    l1 = manipulator.l1
    l2 = manipulator.l2
    l3 = manipulator.l3

    for i in range(l):
        x, y, z = r[:, i]

        q[l, 0] = np.arctan2(x, y)
        rho = np.sqrt(x**2 + y**2)

        q[l, 2] = ((rho**2 + r[2]**2) - (l2**2 + l3**2)) / (2 * l2 * l3)

        alpha = np.arctan(r[2]-l1/rho)
        beta = np.arctan((l2 * np.sin(q[l, 2]))/ (l1 + l2 * np.cos(q[l, 2])))
        q[l, 1] = alpha + beta

    for i in range(l):
        # Position articulaire par cinématique inverse
        q[:, i] = manipulator.inverse_kinematics(r[:, i])

    for i in range(l):
        # Jacobienne en configuration courante
        J = manipulator.jacobian(q[:, i])
        dq[:, i] = np.linalg.pinv(J) @ dr[:, i]

    for i in range(1, l - 1):
        # Approximation de la dérivée par différence centrée
        ddq[:, i] = (dq[:, i + 1] - dq[:, i - 1]) / ((r.shape[1] / 1000) * manipulator.tfinal)

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

    for i in range(l):
        x = np.concatenate([q[:, i], dq[:, i]])  # état complet
        u = ddq[:, i]                            # accélération désirée
        tau[:, i] = manipulator.compute_control(x, u, t=0)

    return tau


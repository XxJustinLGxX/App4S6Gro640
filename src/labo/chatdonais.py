import pyro
import numpy as np

from pyro.dynamic import manipulator
from pyro.control  import robotcontrollers

# ------------------------------------------------------------------
# 1) Modèle cinématique du bras à deux maillons
# ------------------------------------------------------------------
torque_controlled_robot = manipulator.TwoLinkManipulator()

# Longueurs des liens (en unités arbitraires)
torque_controlled_robot.l1 = 4.0
torque_controlled_robot.l2 = 3.0
torque_controlled_robot.l_domain = 10.0   # utile pour l’affichage

# On passe au modèle purement cinématique (entrées = vitesses articulaires)
robot = manipulator.SpeedControlledManipulator.from_manipulator(
    torque_controlled_robot
)

# État initial : [θ1, θ2] (rad)
robot.x0 = np.array([0.1, 0.1])

# ------------------------------------------------------------------
# 2) Contrôleur : suivi point-par-point dans le plan XY
# ------------------------------------------------------------------
class WaypointKinematicController(robotcontrollers.EndEffectorKinematicController):
    def __init__(self, model):
        super().__init__(model)

        # Points cibles (XY)
        self.waypoints = np.array([
            [5.0, 0.0],
            [3.0, 0.0],
            [0.0, 3.0],
            [0.0, 5.0]
        ])

        # Paramètres temporels
        self.total_time   = 6.0
        self.seg_count    = len(self.waypoints) - 1
        self.seg_time     = self.total_time / self.seg_count

    # ----- Outils internes -------------------------------------------------
    def _segment_index(self, t):
        return min(int(t / self.seg_time), self.seg_count - 1)

    def _desired_pos_vel(self, t):
        i   = self._segment_index(t)
        t0  = i * self.seg_time
        t1  = (i + 1) * self.seg_time
        α   = (t - t0) / (t1 - t0)

        p0, p1 = self.waypoints[i], self.waypoints[i + 1]

        r_d  = (1 - α) * p0 + α * p1          # position souhaitée
        dr_d = (p1 - p0) / (t1 - t0)          # vitesse constante

        return r_d, dr_d

    # ----- Loi de commande -------------------------------------------------
    def c(self, y, _r_unused, t):
        q   = y                       # angles articulaires mesurés
        J   = self.J(q)               # Jacobien
        r   = self.fwd_kin(q)         # position actuelle de l’effecteur

        r_d, dr_d = self._desired_pos_vel(t)

        e     = r_d - r               # erreur de position XY
        dr_r  = dr_d + self.gains * e # vitesse eff. désirée (P + feed-fwd)

        dq = np.linalg.pinv(J) @ dr_r # vitesse articulaire
        return dq

# ------------------------------------------------------------------
# 3) Boucle fermée & simulation
# ------------------------------------------------------------------
ctl = WaypointKinematicController(robot)
ctl.gains = np.array([5.0, 5.0])      # gains proportionnels

robot_cl = ctl + robot                # système bouclé
robot_cl.x0 = np.array([1.0, 1.0])    # config. initiale choisie

# Simulation 0 → 6 s
robot_cl.compute_trajectory(tf=6.0)

# ------------------------------------------------------------------
# 4) Visualisation
# ------------------------------------------------------------------
robot_cl.animate_simulation()   # animation du bras
robot_cl.plot_trajectory('x')   # angles articulaires
robot_cl.plot_trajectory('u')   # vitesses articulaires

class CustomKinematicController(robotcontrollers.EndEffectorKinematicController):
    def __init__(self, robot):
        super().__init__(robot)
        self.trajectoire = np.array([[5, 0], [3, 0], [0, 3], [0, 5]])
        self.index_live = 0
        self.kp = 2

    #############################
    def c(self, y, r, t):
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
        q = y  # Joint angles

        # Pre-computed values based on the robot kinematic model
        J = self.J(q)  # Jacobian computation
        r = self.fwd_kin(q)  # End-effector postion

        ##############################
        # YOUR CODE BELLOW !!
        ##############################

        # Compute the reference
        position_error = self.trajectoire[self.index_live] - r
        dr_d = self.kp * position_error
        if np.linalg.norm(position_error) < 0.1 and self.index_live < 3:
            self.index_live = self.index_live + 1

        # From effector speed to joint speed
        dq = np.linalg.inv(J) @ dr_d

        return dq
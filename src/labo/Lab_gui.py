import numpy as np
import matplotlib.pyplot as plt
from pyro.dynamic import manipulator
from mpl_toolkits.mplot3d import Axes3D  # Nécessaire pour les surfaces 3D


def main():
    # --- Création du robot dynamique ---
    robot = manipulator.TwoLinkManipulator()
    robot.l1 = 3.0
    robot.l2 = 3.0

    # --- Force appliquée à l'effecteur ---
    f_ext = np.array([[0.0], [-1.0]])  # 1 N vers le bas

    # --- Grille d'angles ---
    n = 100
    q1_vals = np.linspace(-np.pi, np.pi, n)
    q2_vals = np.linspace(-np.pi, np.pi, n)
    Q1, Q2 = np.meshgrid(q1_vals, q2_vals)

    tau1 = np.zeros_like(Q1)
    tau2 = np.zeros_like(Q2)

    # --- Calcul des couples pour chaque configuration ---
    for i in range(n):
        for j in range(n):
            q = np.array([Q1[i, j], Q2[i, j]])
            J = robot.kinematic.jacobian(q)
            tau = J.T @ f_ext  # τ = Jᵗ * f
            tau1[i, j] = tau[0, 0]
            tau2[i, j] = tau[1, 0]

    # --- Affichage des surfaces 3D ---
    fig = plt.figure(figsize=(12, 5))

    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.plot_surface(Q1, Q2, tau1, cmap='viridis')
    ax1.set_title("τ₁ requis pour résister à 1N vers le bas")
    ax1.set_xlabel("q₁ (rad)")
    ax1.set_ylabel("q₂ (rad)")
    ax1.set_zlabel("τ₁ (Nm)")

    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax2.plot_surface(Q1, Q2, tau2, cmap='plasma')
    ax2.set_title("τ₂ requis pour résister à 1N vers le bas")
    ax2.set_xlabel("q₁ (rad)")
    ax2.set_ylabel("q₂ (rad)")
    ax2.set_zlabel("τ₂ (Nm)")

    plt.tight_layout()
    plt.show()

    # Optionnel : enregistrer la figure
    # fig.savefig("torques_surface.png", dpi=300)

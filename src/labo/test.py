import numpy as np

r = 0.055
d = 0.006
T = np.zeros((4, 4), dtype=np.float64)
theta = 0.3
alpha = np.pi
###################
    # Votre code ici
    ###################
    #colonne 1
T[0][0] = np.cos(theta)
T[0][1] = np.sin(theta)
T[0][2] = 0.0
T[0][3] = 0.0
  #colonne 2
T[1][0] = -np.sin(theta)*np.cos(alpha)
T[1][1] = np.cos(theta)*np.cos(alpha)
T[1][2] = np.sin(alpha)
T[1][3] = 0.0
    #colonne 3
T[2][0] = np.sin(theta)*np.sin(alpha)
T[2][1] = -np.cos(theta)*np.sin(alpha)
T[2][2] = np.cos(alpha)
T[2][3] = 0.0
    #colonne 4
T[3][0] = r*np.cos(theta)
T[3][1] = r*np.sin(theta)
T[3][2] = d
T[3][3] = 1.0

print(T)
import numpy as np

#print(np.array([[100.0, 0.0, 0.0],[0.0, 100.0, 0.0],[0.0, 0.0, 100.0]]))

#Fe = np.zeros((3,1))
#Fe[2] = -200
#print(Fe)

r_0 = np.array([  0.5,   0.0,   1.0]) # start
r_f = np.array([ -0.25, -0.4,   0.6]) # end-point
t_f = 3.0                             # duration

# Time discretization
l = 1000  # nb of time steps
    # Number of DoF for the effector only
m = 3

r = np.zeros((m, l))
dr = np.zeros((m, l))
ddr = np.zeros((m, l))

t = np.linspace(0, t_f, l)  # vecteur temps

# Chemin géométrique (ligne droite)
delta_r = r_f - r_0

# Profil temporel s(t), ds(t), dds(t) 
s = 3 * ( (t / t_f) ** 2 ) - 2 * ( (t / t_f) ** 3 )      
ds = 6* (t/ (t_f**2)) - 6* ( (t**2) / (t_f**3) )         
dds = ( 6 / (t_f**2) ) - ( (12*t) / (t_f**3) )

for i in range(l):
    r[:, i]   = r_0 + s[i] * delta_r            
    dr[:, i]  = ds[i]  * delta_r                 
    ddr[:, i] = dds[i] * delta_r 

print(r[:, l-1])
#print(dr)
#print(ddr)

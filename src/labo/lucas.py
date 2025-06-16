def goal2r( r_0 , r_f , t_f ):
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
    l = 1000 # nb of time steps
   
    # Number of DoF for the effector only
    m = 3
   
    r = np.zeros((m,l))
    dr = np.zeros((m,l))
    ddr = np.zeros((m,l))
 
   
   
    #################################
    # Votre code ici !!!
    ##################################
    """
    rd = r_f-r_0
    r_pos = rd/l
    for i in range(0,l):
        r[0][i] = r_0[0] + (r_pos[0]*(i+1))
        r[1][i] = r_0[1] + (r_pos[1]*(i+1))
        r[2][i] = r_0[2] + (r_pos[2]*(i+1))
        print(r[0][i])
 
        dr[0][i] = r_pos[0]/(t_f/l)
        dr[1][i] = r_pos[1]/(t_f/l)
        dr[2][i] = r_pos[2]/(t_f/l)
        print(dr[0][i])
       
        if(i == l-1):
            ddr[0][i] = (0 - dr[0][i-1])/(t_f/l)
            ddr[1][i] = (0 - dr[1][i-1])/(t_f/l)
            ddr[2][i] = (0 - dr[2][i-1])/(t_f/l)
        if(i == 0):
            ddr[0][i] = (dr[0][i] - 0)/(t_f/l)
            ddr[1][i] = (dr[1][i] - 0)/(t_f/l)
            ddr[2][i] = (dr[2][i] - 0)/(t_f/l)
        elif(i>0):
            ddr[0][i] = (dr[0][i] - dr[0][i-1])/(t_f/l)
            ddr[1][i] = (dr[1][i] - dr[1][i-1])/(t_f/l)
            ddr[2][i] = (dr[2][i] - dr[2][i-1])/(t_f/l)
        print(ddr[0][i])
        #print(r_f-r_0)
    """
    s = np.zeros((l,1))
    ds = np.zeros((l,1))
    dds = np.zeros((l,1))
 
    for i in range(0,l):
        t = (t_f/l)*(i+1)
        s[i] = ((3/(t_f*t_f))*(t*t)) - ((2/(t_f*t_f*t_f))*(t*t*t))
        ds[i] = ((6/(t_f*t_f))*t) - ((6/(t_f*t_f*t_f))*(t*t))
        dds[i] = ((6/(t_f*t_f))) - ((12/(t_f*t_f*t_f))*t)
 
        r[0][i] = r_0[0] + ((r_f[0] - r_0[0]) * s[i])
        r[1][i] = r_0[1] + ((r_f[1] - r_0[1]) * s[i])
        r[2][i] = r_0[2] + ((r_f[2] - r_0[2]) * s[i])
 
        dr[0][i] = ((r_f[0] - r_0[0]) * ds[i])
        dr[1][i] = ((r_f[1] - r_0[1]) * ds[i])
        dr[2][i] = ((r_f[2] - r_0[2]) * ds[i])
 
        ddr[0][i] = ((r_f[0] - r_0[0]) * dds[i])
        ddr[1][i] = ((r_f[1] - r_0[1]) * dds[i])
        ddr[2][i] = ((r_f[2] - r_0[2]) * dds[i])
   
    return r, dr, ddr
 
 
def r2q( r, dr, ddr , manipulator ):
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
    q = np.zeros((n,l))
    dq = np.zeros((n,l))
    ddq = np.zeros((n,l))
   
    #################################
    # Votre code ici !!!
    ##################################
    l1  = manipulator.l1
    l2  = manipulator.l2
    l3  = manipulator.l3
 
    for i in range(0,l):
        x = r[0][i]
        y = r[1][i]
        z = r[2][i]
 
        q[0][i] = np.arctan2(x, y)
        q[2][i] = np.arccos(((((x*x)+(y*y)) + ((z-l1)*(z-l1))) - ((l2*l2)+(l3*l3)))/(2*l2*l3))
        q[1][i] = np.arctan2((z-l1), np.sqrt((x*x)+(y*y))) + np.arctan2((l2*np.sin(q[2][i])),(l1+(l2*np.sin(q[2][i]))))
 
        J = manipulator.J(q[:,i])
        dq[:,i] = np.matmul(np.linalg.inv(J), dr[:,i])
        if i>0 and i<l-1:
            J_plus = manipulator.J(q[:,i+1])
            J_moins = manipulator.J(q[:,i-1])
            DJ = J_plus-J_moins/((i+1)-(i-1))
 
            ddq[:,i] = np.matmul(np.linalg.inv(J), ddr[:,i]) - np.matmul(np.linalg.inv(J),(np.matmul(DJ, dq[:,i])))
   
 
   
   
    return q, dq, ddq
 
 
 
def q2torque( q, dq, ddq , manipulator ):
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
    tau = np.zeros((n,l))
   
    #################################
    # Votre code ici !!!
    ##################################
   
   
    return tau
 
"""
d1 = 0
r1 = 0
l2 = 0
l3 = 0
l5 = 0
l6 = 0
 
q1 =1
q2 =1
q3 =1
q4 =1
q5 =1
 
 
dh_array = np.zeros((3,4))
dh_array = np.array([[d1,r1,q1,-1*np.pi/2],
                     [0,l2,q2,0],
                     [0,l3,q3,0],
                     [0,0,q4,-1*np.pi/2],
                     [l5,0,q5,np.pi/2],
                     [0,l6,0,0]])
 
print(dh_array)
 
print(dhs2T(dh_array[:,0],dh_array[:,1],dh_array[:,2],dh_array[:,3]))
"""
 
q_array = np.zeros((6,1))
 
q_array[0] = 0
q_array[1] = 0
q_array[2] = 0
q_array[3] = 0
q_array[4] = 0
q_array[5] = 0.01
 
print(f(q_array))
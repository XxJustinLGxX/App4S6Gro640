import numpy as np

print(np.array([[100.0, 0.0, 0.0],[0.0, 100.0, 0.0],[0.0, 0.0, 100.0]]))

Fe = np.zeros((3,1))
Fe[2] = -200
print(Fe)

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
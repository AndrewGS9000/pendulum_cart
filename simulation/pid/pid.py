import numpy as np
from scipy import linalg
from scipy.signal import place_poles


class PID:
    def __init__(self, dynamics, P, I, D, time_step):
        self.dynamics = dynamics
        
        # PID parameters
        self.P = P
        self.I = I
        self.D = D
        self.time_step = time_step
        self.integral = 0

        self.alpha = 0.2  # Smoothing factor
        self.previous_u = 0  # Initialize previous control force
        
        
        # Threshold for switching to PID
        #self.lqr_threshold = 0.6 
        
        # get linearized system
        #self.A , self.B = self.dynamics.get_linearized_system()
        #self.C = np.array([[1, 0, 0, 0],[0, 0, 1, 0]])

        # Pre-compute LQR gain for upright position
        # self.K_lqr = self._compute_lqr_gain()
        # self.L = self.compute_L()
    
    
    # def _compute_lqr_gain(self):
    #     """Compute LQR gain for linearized system at upright position"""
    #     # A , B = self.A, self.B
    #     # P = linalg.solve_continuous_are(A, B, self.Q, self.R)
    #     # K = -np.linalg.inv(self.R) @ B.T @ P


    #     return K


    def control(self, state, state_old, target):
        """
        Control algorithm for PID controller.

        state -> [cart_x, theta]
        old_state -> [old_cart_x, old_theta]
        target -> usually only one target as we want cart_x = theta = 0
        """
        theta = state[1]
        old_theta = state_old[1]
        cart_x = state[0]
        old_cart_x = state_old[0]


        # theta and cart_x as targets
        # err = -(cart_x - target) * 0.001  + (theta - target)
        # old_err = -(old_cart_x - target) * 0.001 + (old_theta - target)
        
        # only theta as target
        err = (theta - target)
        old_err = (old_theta - target)

        self.integral += err

        u = self.P * err + (self.I * self.integral) + self.D * ((err - old_err) / (self.time_step))
        #u = np.clip(u, -100, 100) #safety

        u_smoothed = self.alpha * self.previous_u + (1 - self.alpha) * u
        self.previous_u = u_smoothed  # Update for next step
        return u_smoothed
        #return u
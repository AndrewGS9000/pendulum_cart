import numpy as np
from scipy import linalg
from scipy.signal import place_poles

class Controller:
    def __init__(self, dynamics):
        self.dynamics = dynamics
        
        # FBL control gains
        self.k2 = 3.1  # Inner loop proportional gain
        self.k1 = 2 * np.sqrt(self.k2)  # Inner loop derivative gain (critical damping)
        self.k3 = 1.2  # Outer loop position proportional gain
        self.k4 = 0.2 # Outer loop position derivative gain
        
        self.alpha = 0.7  # Smoothing factor
        self.previous_u = 0  # Previous control force
        
        # Observer setup
        self.A, self.B = self.dynamics.get_linearized_system()
        self.C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        
        
        # Observer gain matrix
        self.L = self._compute_observer_gain()
        
        self.max_force = 10
        self.trajectory_time = 20.0
        
        
    
    def control(self, state):
        """Compute FBL control input"""
        x, x_dot, theta, theta_dot = state
        M = self.dynamics.M
        m = self.dynamics.m
        l = self.dynamics.l
        g = self.dynamics.g
        c = self.dynamics.c
        d = self.dynamics.d
        target_x = self.dynamics.target_x
        
        # Outer loop: Generate reference angle for cart position tracking
        theta_ref = (self.k3 * (target_x - x) - self.k4 * x_dot) / g

        # Inner loop: Feedback linearization control law
        F = (M * l * (self.k1 * theta_dot + self.k2 * (theta - theta_ref)) 
             + (M + m) * g * theta)

        # Smoothing and clipping
        F_smoothed = self.alpha * self.previous_u + (1 - self.alpha) * F
        self.previous_u = F_smoothed
        return np.clip(F_smoothed, -self.max_force, self.max_force)
        
    
    def _compute_observer_gain(self):
        """Compute observer gain using pole placement"""
        desired_poles = [-50, -15, -80, -15]  # Faster than controller poles
        result = place_poles(self.A.T, self.C.T, desired_poles)
        return result.gain_matrix.T

    def observer(self, x_hat, y, u, dt):
        """Update state estimate using Luenberger observer"""
        y_hat = self.C @ x_hat
        dx_hat = self.A @ x_hat + self.B @ np.array([u])+ self.L @ (y - y_hat)
        return x_hat + dx_hat * dt

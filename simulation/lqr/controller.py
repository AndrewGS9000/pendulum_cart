import numpy as np
from scipy import linalg
from scipy.signal import place_poles

class Controller:
    def __init__(self, dynamics):
        self.dynamics = dynamics
        
        # LQR parameters
        self.Q = np.diag([10, 1, 10, 1])
        self.R = np.array([[0.1]])

        self.alpha = 0.7  # Smoothing factor
        self.previous_u = 0  # Initialize previous control force
        
        
        # Threshold for switching to LQR
        self.lqr_threshold = 0.6 
        
        # get linearized system
        self.A , self.B = self.dynamics.get_linearized_system()
        self.C = np.array([[1, 0, 0, 0],[0, 0, 1, 0]])

        # Pre-compute LQR gain for upright position
        self.K_lqr = self._compute_lqr_gain()
        self.L = self.compute_L()
    
    
    def _compute_lqr_gain(self):
        """Compute LQR gain for linearized system at upright position"""
        A , B = self.A, self.B
        P = linalg.solve_continuous_are(A, B, self.Q, self.R)
        K = -np.linalg.inv(self.R) @ B.T @ P
        return K


    def control(self, state):
        """Two-phase control: swing-up and stabilization"""
        theta = state[2]
        dtheta = state[3]
        theta_wrapped = (theta + np.pi) % (2 * np.pi) - np.pi  # Wrap to [-π, π]
        cart_x = state[0]
        cart_dx = state[1]

        near_upright = abs(theta_wrapped - 0) < self.lqr_threshold
        # Check if near upright position
        if near_upright:
            # LQR stabilization
            # state_error = state - np.array([0, 0, 0, 0])
            u = float(self.K_lqr @ state)
            u = np.clip(u, -10, 10)

            u_smoothed = self.alpha * self.previous_u + (1 - self.alpha) * u
            self.previous_u = u_smoothed  # Update for next step

            return u_smoothed
        
        else:   
                # position control  within ±0.5 m
                k_p_cart = 20.0 
                k_d_cart = 8.0   
                position_bound = 0.5
                position_control = -k_p_cart * cart_x - k_d_cart * cart_dx
                
                # calculate phase of our system
                phase = np.arctan2(dtheta, theta)  
                
                # input gain
                k_pump = 30.0  
                
                # the best phase to inject energy
                pump_phase = np.pi/2  
                
                # 计算与理想相位的距离
                phase_diff = abs(phase - pump_phase) % (2*np.pi)
                
                # inject at proper time window
                phase_window = np.pi/4  
                
                u_pump = 0
                if phase_diff < phase_window or phase_diff > 2*np.pi - phase_window:
                    # inject energy
                    u_pump = k_pump * np.sign(dtheta)
                
                # compose input
                u = position_control + u_pump
                
                # set input limit
                max_u = 40.0 * (1 - 0.6 * abs(cart_x/position_bound))
                return np.clip(u, -max_u, max_u)
            

    
    def compute_L(self):
        # Desired poles for the observer
        desired_observer_poles = [-50, -15, -80, -15]

        # Place poles and compute L
        L =  place_poles(self.A.T, self.C.T, desired_observer_poles).gain_matrix.T
        return L


    def observer(self, x_hat, y, u, dt):
        
        A,B = self.A, self.B

        # Observer dynamics
        y_hat = self.C @ x_hat
        x_hat_dot = A @ x_hat + B @ np.array([u]) + self.L @ (y - y_hat)
        x_hat = x_hat + dt * x_hat_dot

        return x_hat
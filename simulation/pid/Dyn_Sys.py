import numpy as np

class PendulumCartSystem:
    def __init__(self, M=1.0, m=0.1, l=0.5, g=9.81, d=0.01, c=0.01, target_x=0):
        self.M = M  # cart mass
        self.m = m  # pendulum mass
        self.l = l  # pendulum length
        self.g = g  # gravity
        self.d = d  # cart damping
        self.c = c  # pendulum damping
        self.target_x = target_x
        
        # State vector: [x, theta, x_dot, theta_dot]
        self.n_states = 4


    def state_derivative(self, state, u):
        """
        Compute the derivatives of the state vector [x, x_dot, theta, theta_dot].
        
        Parameters:
            state: array_like, shape (4,)
                The state vector [x, x_dot, theta, theta_dot].
            u: float
                The input force applied to the cart (F).
        
        Returns:
            dstate: array_like, shape (4,)
                The time derivatives [x_dot, x_ddot, theta_dot, theta_ddot].
        """
        x, x_dot, theta, theta_dot = state
        L = self.l
        M = self.M
        m = self.m
        g = self.g
        d = self.d
        c = self.c
        # Common terms
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        # D = m*L*L*(M + m * (1-cos_theta**2))  # Denominator term
        D = (M + m * sin_theta**2)  # Denominator term

        # Accelerations
        x_ddot = (1/D)*( -m*g*cos_theta*sin_theta + c/L*cos_theta*theta_dot + m*L*sin_theta*theta_dot**2 - d*x_dot + u) 

        theta_ddot = (1/D)*((M + m)*g/L*sin_theta - m*sin_theta*cos_theta*theta_dot**2 + d*cos_theta/L*x_dot - c*(M+m)/(m*L**2)*theta_dot - cos_theta/L*u) 


        # Derivatives of state variables
        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])


    def get_linearized_system(self):
        """
        Get linearized system matrices at the upright equilibrium point (θ = 0)
        """
        M, m, L, g, d, c = self.M, self.m, self.l, self.g, self.d, self.c
 
        
        # State matrix A (4x4)
        A = np.array([
            [0,                1,                  0,              0],
            [0,              -d/M,            -m*g/M,             0],
            [0,               0,                  0,              1],
            [0,           -d/(L*M),     (M+m)*g/(L*M),    -c*(M+m)/(m*L*L*M)]
        ])
        
        # Input matrix B (4x1)
        B = np.array([
            [0],
            [1/M],
            [0],
            [-1/(L*M)]
        ])
        
        return A, B


    def add_process_noise(self):
        """measurement noise (guassian distribution)"""
        # Noise for x 1cm
        # (common position sensor pricise is lower than 1cm, so this is enough)
        noise_x = np.random.normal(0, 0.01)  
        # Noise for theta 1 degree
        # (common degree sensor pricise is lower than 0.5degree, so this is enough)
        noise_theta = np.random.normal(0, 0.017)  
        noise = np.array([noise_x, noise_theta])
        return noise



    
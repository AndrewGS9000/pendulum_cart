import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from scipy.signal import place_poles


class PendulumCartSystem:
    def __init__(self, M=1.0, m=0.1, l=0.5, g=9.81, d=0.01, c=0.01):
        self.M = M
        self.m = m
        self.l = l
        self.g = g
        self.d = d
        self.c = c
        self.n_states = 4

    def state_derivative(self, state, u):
        x, x_dot, theta, theta_dot = state
        L, M, m, g, d, c = self.l, self.M, self.m, self.g, self.d, self.c
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        D = (M + m * sin_theta**2)

        x_ddot = (1/D)*(-m*g*cos_theta*sin_theta + c/L*cos_theta*theta_dot +
                        m*L*sin_theta*theta_dot**2 - d*x_dot + u)
        theta_ddot = (1/D)*(((M + m)*g/L*sin_theta) - m*sin_theta*cos_theta*theta_dot**2 +
                            d*cos_theta/L*x_dot - c*(M+m)/(m*L**2)*theta_dot - cos_theta/L*u)
        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

    def get_linearized_system(self):
        M, m, L, g, d, c = self.M, self.m, self.l, self.g, self.d, self.c
        A = np.array([
            [0, 1, 0, 0],
            [0, -d/M, -m*g/M, 0],
            [0, 0, 0, 1],
            [0, -d/(L*M), (M+m)*g/(L*M), -c*(M+m)/(m*L*L*M)]
        ])
        B = np.array([
            [0],
            [1/M],
            [0],
            [-1/(L*M)]
        ])
        return A, B

    def add_process_noise(self):
        noise_x = np.random.normal(0, 0.01)
        noise_theta = np.random.normal(0, 0.017)
        return np.array([noise_x, noise_theta])


class HybridController:
    def __init__(self, dynamics, control_mode='pid', dt=0.01,
                 pid_params=(20, 0.05, 1),
                 lqr_Q=np.diag([10, 1, 10, 1]),
                 lqr_R=np.array([[0.1]]),
                 use_nonlinear_comp=False):
        self.dynamics = dynamics
        self.control_mode = control_mode.lower()
        self.dt = dt
        self.use_nonlinear_comp = use_nonlinear_comp

        # PID
        self.P, self.I, self.D = pid_params
        self.integral = 0
        self.alpha_pid = 0.2
        self.prev_u_pid = 0

        # Linear system
        self.A, self.B = self.dynamics.get_linearized_system()
        self.C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

        # LQR
        self.Q = lqr_Q
        self.R = lqr_R
        self.K_lqr = self._compute_lqr_gain()
        self.L = self.compute_L()
        self.alpha_lqr = 0.7
        self.prev_u_lqr = 0

        # Pole Placement
        self.K_pole = self._compute_pole_placement_gain()
        self.alpha_pole = 0.5
        self.prev_u_pole = 0

    def _compute_lqr_gain(self):
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = -np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def compute_L(self):
        desired_poles = [-50, -15, -80, -15]
        L = place_poles(self.A.T, self.C.T, desired_poles).gain_matrix.T
        return L

    def _compute_pole_placement_gain(self):
        desired_poles = [-2.0, -2.5, -3.0, -3.5]
        placed = place_poles(self.A, self.B, desired_poles)
        return placed.gain_matrix

    def control(self, state, state_old=None, target=0):
        if self.control_mode == 'pid':
            return self._pid_control(state, state_old, target)
        elif self.control_mode == 'non_linear':
            return self._lqr_control(state)
        elif self.control_mode == 'pole':
            return self._pole_control(state)
        else:
            raise ValueError(f"Unknown control mode: {self.control_mode}")

    def _pid_control(self, state, state_old, target):
        theta, old_theta = state[1], state_old[1]
        err = theta - target
        old_err = old_theta - target
        self.integral += err

        u = self.P * err + self.I * self.integral + self.D * ((err - old_err) / self.dt)
        u_smoothed = self.alpha_pid * self.prev_u_pid + (1 - self.alpha_pid) * u
        self.prev_u_pid = u_smoothed
        return u_smoothed

    def _lqr_control(self, state):
        x, x_dot, theta, theta_dot = state
        u_lqr = float(self.K_lqr @ state)

        if self.use_nonlinear_comp:
            m = self.dynamics.m
            M = self.dynamics.M
            L = self.dynamics.l
            g = self.dynamics.g

            sin_theta = np.sin(theta)
            cos_theta = np.cos(theta)

            nonlinear_comp = (
                m * g * sin_theta * cos_theta
                - m * L * sin_theta * theta_dot**2
                + (M + m * sin_theta**2) * (g * sin_theta / L)
            )
            u = u_lqr + nonlinear_comp
        else:
            u = u_lqr

        u = np.clip(u, -15, 15)
        u_smoothed = self.alpha_lqr * self.prev_u_lqr + (1 - self.alpha_lqr) * u
        self.prev_u_lqr = u_smoothed
        return u_smoothed

    def _pole_control(self, state):
        u = float(-self.K_pole @ state)
        u = np.clip(u, -15, 15)
        u_smoothed = self.alpha_pole * self.prev_u_pole + (1 - self.alpha_pole) * u
        self.prev_u_pole = u_smoothed
        return u_smoothed

    def observer(self, x_hat, y, u):
        y_hat = self.C @ x_hat
        x_hat_dot = self.A @ x_hat + self.B @ np.array([u]) + self.L @ (y - y_hat)
        return x_hat + self.dt * x_hat_dot


class Simulator:
    def __init__(self, dynamics, controller):
        self.dynamics = dynamics
        self.controller = controller
        self.C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

    def simulate(self, x0, t_span, dt=0.01):
        t = np.arange(0, t_span, dt)
        n = len(t)
        x = np.zeros((self.dynamics.n_states, n))
        x_hat = np.zeros((self.dynamics.n_states, n))
        u = np.zeros(n)
        noise = np.zeros((2, n))
        target = 0

        x[:, 0] = x0
        x_hat[:, 0] = x0
        old_measured = [x0[0], x0[2]]

        for i in range(n - 1):
            noise[:, i] = self.dynamics.add_process_noise()
            x[:, i+1] = x[:, i] + dt * self.dynamics.state_derivative(x[:, i], u[i])

            cart_x_measured = x[0, i+1] + noise[0, i]
            theta_measured = x[2, i+1] + noise[1, i]
            measured = [cart_x_measured, theta_measured]

            if self.controller.control_mode == 'pid':
                u[i+1] = self.controller.control(measured, old_measured, target)
                old_measured = measured
            else:
                full_state = x[:, i+1]
                u[i+1] = self.controller.control(full_state)

            if i == int(5 / dt):
                x[:, i+1][2] += np.deg2rad(4)

        return t, x, u, x_hat, noise

    def plot_results(self, t, x, u, x_hat, noise):
        fig, axs = plt.subplots(4, 1, figsize=(10, 15))

        axs[0].plot(t, x[0, :], 'b-', label='Cart Position')
        axs[0].set_ylabel('Position (m)')
        axs[0].legend()
        axs[0].grid()

        axs[1].plot(t, np.rad2deg(x[2, :]), 'r-', label='Pendulum Angle')
        axs[1].set_ylabel('Angle (deg)')
        axs[1].legend()
        axs[1].grid()

        axs[2].plot(t, noise[1, :], 'g-', label='Angle Noise')
        axs[2].set_ylabel('Noise (rad)')
        axs[2].legend()
        axs[2].grid()

        axs[3].plot(t, u, 'k-', label='Input Force')
        axs[3].set_xlabel('Time (s)')
        axs[3].set_ylabel('Force (N)')
        axs[3].legend()
        axs[3].grid()

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    x0 = np.array([0.1, 0.0, 0.3, 0.0])  # 初始状态
    t_span = 10
    dt = 0.01

    system = PendulumCartSystem()

    # 启用带非线性补偿的 LQR 控制器
    controller = HybridController(system, control_mode='non_linear', dt=dt, use_nonlinear_comp=True)

    simulator = Simulator(system, controller)
    t, x, u, x_hat, noise = simulator.simulate(x0, t_span, dt)
    simulator.plot_results(t, x, u, x_hat, noise)

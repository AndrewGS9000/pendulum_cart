import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from scipy.signal import place_poles
import tkinter as tk
from tkinter import ttk

# ------------------ system and controller ------------------

class PendulumCartSystem:
    def __init__(self, M=1.0, m=0.1, l=0.5, g=9.81, d=0.01, c=0.01):
        self.M = M
        self.m = m
        self.l = l
        self.g = g
        self.d = d      # cart damping
        self.c = c      # pendulum damping
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
        noise_x = np.random.normal(0, 0.02)
        noise_theta = np.random.normal(0, 0.085)
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

        # linearize
        self.A, self.B = self.dynamics.get_linearized_system()
        self.C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

        # LQR
        self.Q = lqr_Q
        self.R = lqr_R
        self.K_lqr = self._compute_lqr_gain()
        self.L = self.compute_L()
        self.alpha_lqr = 0.7
        self.prev_u_lqr = 0

        # pole
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
        elif self.control_mode == 'non-linear':
            return self._nonlinear_control(state)
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

    def _nonlinear_control(self, state):
        x, x_dot, theta, theta_dot = state
        u_lqr = float(self.K_lqr @ state)
        if self.use_nonlinear_comp:
            m, M, L, g = self.dynamics.m, self.dynamics.M, self.dynamics.l, self.dynamics.g 
            sin_theta = np.sin(theta)
            cos_theta = np.cos(theta)
            nonlinear_comp = (m * g * sin_theta * cos_theta - m * L * sin_theta * theta_dot**2 +
                              (M + m * sin_theta**2) * (g * sin_theta / L))
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
    def __init__(self, dynamics, controller, noise_enabled=True):
        self.dynamics = dynamics
        self.controller = controller
        self.noise_enabled = noise_enabled
        self.C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

    def simulate(self, x0, t_span=10, dt=0.01):
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
            if self.noise_enabled:
                noise[:, i] = self.dynamics.add_process_noise()
            else:
                noise[:, i] = np.zeros(2)
                
            # true state
            x[:, i+1] = x[:, i] + dt * self.dynamics.state_derivative(x[:, i], u[i])
            # noise
            noisy_state = x[:, i+1].copy()
            noisy_state[0] += noise[0, i]
            noisy_state[2] += noise[1, i]

            if self.controller.control_mode == 'pid':
                measured = [noisy_state[0], noisy_state[2]]
                u[i+1] = self.controller.control(measured, old_measured, target)
                old_measured = measured
            else:
                u[i+1] = self.controller.control(noisy_state)

            # disturbance
            if i == int(5 / dt):
                x[:, i+1][2] += np.deg2rad(4)

        return t, x, u, x_hat, noise

    def animate(self, t, x, u):
        import matplotlib.patches as patches
        from matplotlib.animation import FuncAnimation

        fig, (ax_cart, ax_pos, ax_angle, ax_force) = plt.subplots(4, 1, figsize=(10, 10))

        # animation
        ax_cart.set_xlim(-2, 2)
        ax_cart.set_ylim(-0.5, 1.5)
        ax_cart.set_aspect('equal')
        ax_cart.grid()

        cart_width = 0.4
        cart_height = 0.2
        cart_patch = patches.Rectangle((0, 0), cart_width, cart_height, fc='blue')
        pendulum_line, = ax_cart.plot([], [], 'k-', lw=2)
        pendulum_mass = patches.Circle((0, 0), 0.05, fc='red')
        ax_cart.add_patch(cart_patch)
        ax_cart.add_patch(pendulum_mass)

        # plot
        ax_pos.set_ylabel('Cart Pos (m)')
        ax_angle.set_ylabel('Angle (deg)')
        ax_force.set_ylabel('Force (N)')
        ax_force.set_xlabel('Time (s)')

        ax_pos.grid()
        ax_pos.set_xlim(t[0], t[-1])
        ax_pos.set_ylim(-1, 1)
        for ax in (ax_angle, ax_force):
            ax.grid()
            ax.set_xlim(t[0], t[-1])
            ax.set_ylim(-14, 14)

        line_pos, = ax_pos.plot([], [], 'b-')
        line_angle, = ax_angle.plot([], [], 'r-')
        line_force, = ax_force.plot([], [], 'g-')

        def init():
            cart_patch.set_xy((-cart_width/2, 0))
            pendulum_line.set_data([], [])
            pendulum_mass.center = (0, 0)
            return cart_patch, pendulum_line, pendulum_mass, line_pos, line_angle, line_force

        def animate_frame(i):
            cart_x = x[0, i]
            theta = x[2, i]
            cart_patch.set_xy((cart_x - cart_width/2, 0))
            top_of_cart = (cart_x, cart_height)
            pendulum_x = top_of_cart[0] + self.dynamics.l * np.sin(theta)
            pendulum_y = top_of_cart[1] + self.dynamics.l * np.cos(theta)
            pendulum_line.set_data([top_of_cart[0], pendulum_x], [top_of_cart[1], pendulum_y])
            pendulum_mass.center = (pendulum_x, pendulum_y)
            line_pos.set_data(t[:i+1], x[0, :i+1])
            line_angle.set_data(t[:i+1], np.rad2deg(x[2, :i+1]))
            line_force.set_data(t[:i+1], u[:i+1])
            return cart_patch, pendulum_line, pendulum_mass, line_pos, line_angle, line_force

        ani = FuncAnimation(fig, animate_frame, init_func=init, frames=len(t),
                            interval=0.01*1000, blit=True)
        plt.tight_layout()
        plt.show()

# ------------------ GUI ------------------

def run_simulation():
    # get para from gui
    noise_enabled = noise_var.get()
    controller_type = controller_var.get()  
    try:
        d_value = float(d_entry.get())
    except:
        d_value = 0.01
    try:
        c_value = float(c_entry.get())
    except:
        c_value = 0.01
    # pid
    if controller_type == "PID":
        try:
            P_val = float(P_entry.get())
            I_val = float(I_entry.get())
            D_val = float(D_entry.get())
            pid_params = (P_val, I_val, D_val)
        except:
            pid_params = (20, 0.05, 1)
    else:
        pid_params = (20, 0.05, 1)
    # non linear controller
    if controller_type == "Non-linear":
        use_nonlinear_comp = nonlinear_var.get()
    else:
        use_nonlinear_comp = False

    # start a system and controller
    system = PendulumCartSystem(d=d_value, c=c_value)
    controller_mode = controller_type.lower()  # 转换为小写（例如 "pid"）
    controller = HybridController(system, control_mode=controller_mode, dt=0.01,
                                  pid_params=pid_params, use_nonlinear_comp=use_nonlinear_comp)
    simulator = Simulator(system, controller, noise_enabled=noise_enabled)

    # init state
    x0 = np.array([0.1, 0.0, 0.3, 0.0])
    t, x, u, x_hat, noise = simulator.simulate(x0, t_span=10, dt=0.01)
    simulator.animate(t, x, u)


root = tk.Tk()
root.title("Pendulum-Cart Simulation GUI")

noise_var = tk.BooleanVar(value=True)
controller_var = tk.StringVar(value="PID")
nonlinear_var = tk.BooleanVar(value=False)

frame = ttk.Frame(root, padding="10")
frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

ttk.Label(frame, text="Pendulum-Cart Simulation", font=("Arial", 14)).grid(row=0, column=0, columnspan=2, pady=5)

# noise filter
ttk.Checkbutton(frame, text="Disable Noise Filter", variable=noise_var).grid(row=1, column=0, sticky=tk.W, pady=2)

# controller type
ttk.Label(frame, text="Controller Type:").grid(row=2, column=0, sticky=tk.W, pady=2)
ttk.Radiobutton(frame, text="PID", variable=controller_var, value="PID").grid(row=2, column=1, sticky=tk.W)
ttk.Radiobutton(frame, text="Non-linear", variable=controller_var, value="Non-linear").grid(row=3, column=1, sticky=tk.W)
ttk.Radiobutton(frame, text="Pole", variable=controller_var, value="Pole").grid(row=4, column=1, sticky=tk.W)

# damping
ttk.Label(frame, text="Cart damping d (0.001~0.1, default 0.01):").grid(row=5, column=0, sticky=tk.W, pady=2)
d_entry = ttk.Entry(frame)
d_entry.insert(0, "0.01")
d_entry.grid(row=5, column=1, sticky=tk.W)

ttk.Label(frame, text="Pendulum damping c (0.001~0.1, default 0.01):").grid(row=6, column=0, sticky=tk.W, pady=2)
c_entry = ttk.Entry(frame)
c_entry.insert(0, "0.01")
c_entry.grid(row=6, column=1, sticky=tk.W)

# PID para
ttk.Label(frame, text="PID parameters (P, I, D):").grid(row=7, column=0, sticky=tk.W, pady=2)
P_entry = ttk.Entry(frame, width=8)
P_entry.insert(0, "20")
P_entry.grid(row=7, column=1, sticky=tk.W)
ttk.Label(frame, text="P:").grid(row=7, column=0, sticky=tk.E)

I_entry = ttk.Entry(frame, width=8)
I_entry.insert(0, "0.05")
I_entry.grid(row=8, column=1, sticky=tk.W)
ttk.Label(frame, text="I:").grid(row=8, column=0, sticky=tk.E)

D_entry = ttk.Entry(frame, width=8)
D_entry.insert(0, "1")
D_entry.grid(row=9, column=1, sticky=tk.W)
ttk.Label(frame, text="D:").grid(row=9, column=0, sticky=tk.E)

# non linear compensation
ttk.Checkbutton(frame, text="Use Non-linear Compensation", variable=nonlinear_var).grid(row=10, column=0, columnspan=2, sticky=tk.W, pady=2)

# “Run Simulation” 
run_button = ttk.Button(frame, text="Run Simulation", command=run_simulation)
run_button.grid(row=11, column=0, columnspan=2, pady=10)

root.mainloop()

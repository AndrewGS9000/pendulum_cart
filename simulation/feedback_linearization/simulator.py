import numpy as np
from FBL_Dyn_Sys import PendulumCartSystem as Sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from FBL_controller import Controller

class Simulator:
    def __init__(self, dynamics, controller):
        self.dynamics = dynamics
        self.controller = controller
        self.C = np.array([[1, 0, 0, 0],[0, 0, 1, 0]])
    
    
    def simulate(self, x0, t_span, dt=0.01):
        t = np.arange(0, t_span, dt)
        n = len(t)
        x_true = np.zeros((4, n))
        x_est = np.zeros((4, n))
        y = np.zeros((2, n))
        u = np.zeros(n)
        
        x_true[:, 0] = x0
        x_est[:, 0] = x0
        
        noise = np.zeros((2, n))
        
        
        
        for i in range(n-1):
            
            y[:, i] = self.C @ x_true[:, i]
            
            #noise[:, i] = self.dynamics.add_process_noise()
            
            # Measure outputs with noise
            y[:, i] += noise[:, i]
            
            # Update observer
            x_est[:, i+1] = self.controller.observer(x_est[:, i], y[:, i], u[i], dt)
            
            # Compute control using estimate
            u[i+1] = self.controller.control(x_est[:, i+1])
            
            # Propagate true dynamics
            dx = self.dynamics.state_derivative(x_true[:, i], u[i])
            x_true[:, i+1] = x_true[:, i] + dx * dt

            # Safety check
            '''if abs(x_true[0, i+1]) > 15:
                print(f"Cart exceeded limit at t = {t[i]:.2f}s")
                return t[:i+1], x_true[:, :i+1], x_est[:, :i+1], u[:i+1], noise'''
                
        return t, x_true, x_est, u, noise


    def animate(self, t, x):
        fig = plt.figure(figsize=(10, 5))
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-1, 1))
        ax.grid()
        
        # Plot elements
        line, = ax.plot([], [], 'o-', lw=2)
        time_template = 'time = %.1fs'
        time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
        
        def init():
            line.set_data([], [])
            time_text.set_text('')
            return line, time_text
        
        def animate(i):
            cart_x = x[0, i]
            theta = x[1, i]
            
            # Pendulum rod end coordinates
            pendulum_x = cart_x + self.dynamics.l * np.sin(theta)
            pendulum_y = self.dynamics.l * np.cos(theta)
            
            thisx = [cart_x, pendulum_x]
            thisy = [0, pendulum_y]
            
            line.set_data(thisx, thisy)
            time_text.set_text(time_template % t[i])
            return line, time_text
        
        ani = FuncAnimation(fig, animate, len(t), interval=t[1]*1000,
                                    blit=True, init_func=init)
        plt.show()
        return ani


    def plot_results(self, t, x_true, x_est, u, noise):
        fig, axs = plt.subplots(5, 1, figsize=(10, 10))
        
        # Cart position
        axs[0].plot(t, x_true[0], 'b', label='True')
        axs[0].plot(t, x_est[0], 'r--', label='Estimated')
        axs[0].plot(t,[self.dynamics.target_x]*len(x_true[0]), 'g', label='Target')
        axs[0].set_ylabel('Position (m)')
        axs[0].grid(True)
        axs[0].legend()
        
        # Pendulum angle
        axs[1].plot(t, np.rad2deg(x_true[2]), 'b', label='True')
        axs[1].plot(t, np.rad2deg(x_est[2]), 'r--', label='Estimated')
        axs[1].plot(t,[0]*len(x_true[2]), 'g', label='Target')
        axs[1].set_ylabel('Angle (deg)')
        axs[1].grid(True)
        axs[1].legend()
        
        #Control input
        axs[2].plot(t, u, 'g')
        axs[2].set_ylabel('Force (N)')
        axs[2].grid(True)
        
        # Estimation errors
        axs[3].plot(t, x_true[0] - x_est[0], 'k', label='Position Error')
        axs[3].plot(t, x_true[2] - x_est[2], 'm', label='Angle Error')
        axs[3].set_ylabel('Estimation Errors')
        axs[3].grid(True)
        axs[3].legend()
        
        
        # Plot noise
        # axs[3].plot(t, x[0, :], 'r-', label='True angular velocity')
        axs[4].plot(t, noise[1, :], 'g-', label='Position Noise')
        axs[4].set_xlabel('Time (s)')
        axs[4].set_ylabel('Noise (m)')
        axs[4].grid(True)
        axs[4].legend()
        
    
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    # Create system and simulator
    system = Sys()
    controller1 = Controller(system)
    simulator = Simulator(system,controller1)
    
    # Initial conditions: [x, x_dot, theta, theta_dot]
    # max threshold 0.4rad  23 degree
    x0 = np.array([0.0, 0.0, 0.1, 0.0])
    t_span = 25
    dt = 0.01
 
    # Run simulation
    t, x_true, x_est, u, noise = simulator.simulate(x0, t_span, dt)
    
    # Plot results
    simulator.plot_results(t, x_true, x_est, u, noise)
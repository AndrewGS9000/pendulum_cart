import numpy as np
from Dyn_Sys import PendulumCartSystem as Sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from controller import Controller
import matplotlib.patches as patches

class Simulator:
    def __init__(self, dynamics, controller):
        self.dynamics = dynamics
        self.controller = controller
        self.C = np.array([[1, 0, 0, 0],[0, 0, 1, 0]])
    
    
    def simulate(self, x0, t_span, dt=0.01):
        t = np.arange(0, t_span, dt)
        n = len(t)
        x = np.zeros((self.dynamics.n_states, n))
        x_hat = np.zeros((self.dynamics.n_states, n))
        y = np.zeros((2, n))
        u = np.zeros(n)
        x[:, 0] = x0
        x_hat[:, 0] = x0
        noise = np.zeros((2, n))
        disturbance = np.zeros(n)
        disturbance_interval = 1.5 
        disturbance_force = 30.0  # 施加的力大小

        for i in range(n-1):
            
            # measured output
            y[:,i] = self.C @ x[:,i]
            # add measurement noise to simulate real sensor
    
            noise[:, i] = self.dynamics.add_process_noise()

            if (i * dt) % disturbance_interval < dt:
                disturbance[i] = disturbance_force  # 施加外力
            
            y[:,i] = y[:,i] + noise[:, i]
            # estimate full state
            x_hat[:,i+1] = self.controller.observer(x_hat[:,i],y[:,i],u[i],dt)
            # use estimate state to do control
            u[i+1] = self.controller.control(x_hat[:,i+1])
            # true state
            x[:, i+1] = x[:, i] + dt * system.state_derivative(x[:, i],u[i]+ disturbance[i])
            
            # if the cart move out of +/- 15, stop.
            if abs(x[0, i+1]) > 15:
                print(f"Cart position exceeded limit at t = {t[i]:.2f}s")
                return t[:i+1], x[:,:i+1], u[:i+1],  x_hat[:,i+1]
                
        return t, x, u, x_hat, noise, disturbance


    def animate(self, t, x):
        fig = plt.figure(figsize=(10, 5))
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-5, 5), ylim=(-1, 4))
        ax.grid()
        
        # Define cart parameters
        cart_width = 0.5      # cart width
        cart_height = 0.3     # cart height
        cart_y = 0.0          # cart vertical position (bottom of the cart)
        
        # Create cart as a rectangle patch
        cart_patch = patches.Rectangle((0, cart_y), cart_width, cart_height, fc='blue', ec='black')
        ax.add_patch(cart_patch)
        
        # Create pendulum components: a line for the rod and a circle for the mass
        pendulum_line, = ax.plot([], [], 'k-', lw=2)
        mass_radius = 0.05
        pendulum_mass = patches.Circle((0, 0), mass_radius, fc='red', ec='black')
        ax.add_patch(pendulum_mass)
        
        # Time text for display
        time_template = 'time = %.1fs'
        time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes)

        def init():
            cart_patch.set_xy((0 - cart_width/2, cart_y))
            pendulum_line.set_data([], [])
            pendulum_mass.center = (0, 0)
            time_text.set_text('')
            return cart_patch, pendulum_line, pendulum_mass, time_text

        def animate_frame(i):
            # Use correct indices: x[0,i] is cart position, x[2,i] is pendulum angle (θ)
            cart_x = x[0, i]
            theta = x[2, i]
            
            # Update the cart's position (centering the rectangle at cart_x)
            cart_patch.set_xy((cart_x - cart_width/2, cart_y))
            
            # The pendulum's attachment point is the top-center of the cart
            cart_top = (cart_x, cart_y + cart_height)
            
            # For an inverted pendulum, we want θ=0 to have the mass above the cart.
            # So compute the pendulum mass position as:
            pendulum_x = cart_top[0] + self.dynamics.l * np.sin(theta)
            pendulum_y = cart_top[1] + self.dynamics.l * np.cos(theta)
            
            # Update the pendulum rod line (from the cart top to the pendulum mass)
            pendulum_line.set_data([cart_top[0], pendulum_x], [cart_top[1], pendulum_y])
            
            # Update the pendulum mass's position (circle center)
            pendulum_mass.center = (pendulum_x, pendulum_y)
            
            # Update time text
            time_text.set_text(time_template % t[i])
            return cart_patch, pendulum_line, pendulum_mass, time_text

        ani = FuncAnimation(fig, animate_frame, frames=len(t), init_func=init,
                            interval=t[1]*1000, blit=True)
        #plt.show()
        return ani

    def animate_realtime_plots(self, t, x, u):
        fig2, (ax_pos, ax_angle, ax_force) = plt.subplots(3, 1, figsize=(8, 6))
        
        # Configure each subplot
        ax_pos.set_ylabel('Cart Position (m)')
        ax_angle.set_ylabel('Pendulum Angle (deg)')
        ax_force.set_ylabel('Disturbance Force (N)')
        ax_force.set_xlabel('Time (s)')
        ax_pos.set_ylim(-0.4, 1.5)
        ax_angle.set_ylim(-20, 300)
        ax_force.set_ylim(-20, 35)

        for ax in (ax_pos, ax_angle, ax_force):
            ax.grid(True)
            ax.set_xlim(t[0], t[-1])
        
        # Create empty lines for each signal
        line_pos, = ax_pos.plot([], [], 'b-', label='Cart Position')
        line_angle, = ax_angle.plot([], [], 'r-', label='Pendulum Angle')
        line_force, = ax_force.plot([], [], 'g-', label='Disturbance Force')
        
        def init_plots():
            line_pos.set_data([], [])
            line_angle.set_data([], [])
            line_force.set_data([], [])
            return line_pos, line_angle, line_force

        def animate_plots_frame(i):
            line_pos.set_data(t[:i+1], x[0, :i+1])
            # Convert pendulum angle from radians to degrees for display.
            line_angle.set_data(t[:i+1], np.rad2deg(x[2, :i+1]))
            line_force.set_data(t[:i+1], u[:i+1])
            return line_pos, line_angle, line_force

        ani_plots = FuncAnimation(fig2, animate_plots_frame, frames=len(t),
                                init_func=init_plots, interval=t[1]*1000, blit=True)
        return ani_plots


    def plot_results(self, t, x, u, x_hat, noise):
        """Plot the simulation results"""
        fig, axs = plt.subplots(3, 1, figsize=(10, 15))
        
        # Plot cart position
        axs[0].plot(t, x[0, :], 'b-', label='Cart Position')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Position (m)')
        axs[0].grid(True)
        axs[0].legend()
        
        # Plot pendulum angle
        axs[1].plot(t, np.rad2deg(x[2, :]), 'r-', label='Pendulum Angle')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Angle (degrees)')
        axs[1].grid(True)
        axs[1].legend()
        
        # Plot input force
        axs[2].plot(t, u, 'g-', label='Input Force')
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Force (N)')
        axs[2].grid(True)
        axs[2].legend()


        # # Plot position of the pendulum
        # # axs[3].plot(t, x[0, :], 'r-', label='True angular velocity')
        # axs[2].plot(t, x_hat[2, :], 'g-', label='Position Noise')
        # axs[2].set_xlabel('Time (s)')
        # axs[2].set_ylabel('Noise (m)')
        # axs[2].grid(True)
        # axs[2].legend()


        # Plot noise
        # axs[3].plot(t, x[0, :], 'r-', label='True angular velocity')
        # axs[2].plot(t, noise[1, :], 'g-', label='Position Noise')
        # axs[2].set_xlabel('Time (s)')
        # axs[2].set_ylabel('Noise (m)')
        # axs[2].grid(True)
        # axs[2].legend()
            
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    # Create system and simulator
    system = Sys()
    controller1 = Controller(system)
    simulator = Simulator(system,controller1)
    
    # Initial conditions: [x, x_dot, theta, theta_dot]
    # max threshold 0.4rad  23 degree
    x0 = np.array([0.1, 0.0, 3, 0]) 
    t_span = 10
    dt = 0.01
 
    # Run simulation
    t, x, u, x_hat, noise, disturbance = simulator.simulate(x0, t_span, dt)
    ani = simulator.animate(t, x)
    ani_plots = simulator.animate_realtime_plots(t, x, disturbance)
    plt.show()
    #ani.save('cart_pendulum_visualization.mp4', writer='ffmpeg', fps=60)
    #ani_plots.save('realtime_plots.mp4', writer='ffmpeg', fps=60)
    # Plot results
    simulator.plot_results(t, x, u, x_hat, noise)
import numpy as np
from Dyn_Sys import PendulumCartSystem as Sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from controller import Controller
from pid import PID

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
        old_state_measured = [x0[0], x0[2]]
        state_measured = [x0[0], x0[2]]
        x_hat[:, 0] = x0
        noise = np.zeros((2, n))
        target = 0 #here, both theta target and cart_x target is 0 so we only use 1 target
    

        for i in range(n-1):
           
            # add measurement noise to simulate real sensor
            noise[:, i] = self.dynamics.add_process_noise()
        
            #calculate next state for system
            x[:, i+1] = x[:, i] + dt * system.state_derivative(x[:, i],u[i])
            
            #controller 
            cart_x_measured = x[:,i+1][0] + noise[0, i]
            theta_measured = x[:,i+1][2] + noise[1, i]
            old_state_measured = state_measured
            state_measured = [cart_x_measured, theta_measured]

            u[i+1] = self.controller.control(state_measured, old_state_measured, target)
            
            
            # if the cart move out of +/- 15, stop

            # if abs(x[0, i+1]) > 15: 
            #     print(f"Cart position exceeded limit at t = {t[i]:.2f}s")
            #     return t[:i+1], x[:,:i+1], u[:i+1],  x_hat[:,i+1], noise

            #disturbance at t= 5s
            if i == 5 / dt:
                x[:, i+1] = [x[:, i+1][0], x[:, i+1][1], x[:, i+1][2] + 10/60, x[:, i+1][3]] #10 degrees 
                
        return t, x, u, x_hat, noise


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


    def plot_results(self, t, x, u, x_hat, noise):
        """Plot the simulation results"""
        fig, axs = plt.subplots(4, 1, figsize=(10, 15))
        
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
        
        # # Plot input force
        # axs[2].plot(t, u, 'g-', label='Input Force')
        # axs[2].set_xlabel('Time (s)')
        # axs[2].set_ylabel('Force (N)')
        # axs[2].grid(True)
        # axs[2].legend()


        # # Plot position of the pendulum
        # # axs[3].plot(t, x[0, :], 'r-', label='True angular velocity')
        # axs[2].plot(t, x_hat[2, :], 'g-', label='Position Noise')
        # axs[2].set_xlabel('Time (s)')
        # axs[2].set_ylabel('Noise (m)')
        # axs[2].grid(True)
        # axs[2].legend()


        # Plot noise
        # axs[3].plot(t, x[0, :], 'r-', label='True angular velocity')
        axs[2].plot(t, noise[1, :], 'g-', label='Position Noise')
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Noise (m)')
        axs[2].grid(True)
        axs[2].legend()


        # # Plot input force
        axs[3].plot(t, u, 'g-', label='Input Force')
        axs[3].set_xlabel('Time (s)')
        axs[3].set_ylabel('Force (N)')
        axs[3].grid(True)
        axs[3].legend()

            
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    
    
    # Initial conditions: [x, x_dot, theta, theta_dot]
    # if targets = both cart_x and theta, intial theta must be very close to 0 (max about 0.03)
    x0 = np.array([0.1, 0.0, 0.03, 0]) 
    t_span = 10
    dt = 0.01

    # Create system and simulator
    system = Sys()
    controller1 = PID(system, 20, 0.05, 1, dt) #20 0.05 1  #P I D
    simulator = Simulator(system,controller1)
 
    # Run simulation
    t, x, u, x_hat, noise = simulator.simulate(x0, t_span, dt)
    
    # Plot results
    simulator.plot_results(t, x, u, x_hat, noise)
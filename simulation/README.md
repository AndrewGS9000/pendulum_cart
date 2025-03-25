# Simulation code

## features
- **controllers supported:**
  - PID
  - LQR with non-linear component
  - Pole placement
- **real time animation**
- **real time plot**

## How to run:
There is a single python file in this folder containing all of our code for the simulation of the cart.

The code supports 3 different kinds of controllers: PID, non-linear, and pole placement. The code only runs one controller at once. To choose which controller is running, go to line 237 of code ( controller = HybridController(system, control_mode='non_linear', dt=dt, use_nonlinear_comp=True) ) and replace parameter "control_mode=" with your desired controller: 'non_linear' for non-linear, 'pid' for pid and 'pole' for pole placement.

Depending on the controller and computer used, the file may take a moment to run.

## Requirements:
- python 3.11.10
- Visual Studio Code (or another IDE for python)

libraries:
- numpy v1.26.4
- matplotlib v3.9.1
- scipy v1.14.1

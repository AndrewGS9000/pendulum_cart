# Pendulum-Cart Simulation with GUI

This project simulates and visualizes an **inverted pendulum on a cart** system using various control strategies. It provides a simple **GUI (Graphical User Interface)** to select controller parameters, enable/disable noise, and run the simulation with animation and plots.


## How to Run
 - just run the file YAN_simulation_final.py
 - adjust parameters and options in the GUI

## Features

- **Controllers Supported:**
  - PID Controller
  - Linear LQR with Non-linear Compensation
  - Pole Placement Controller
- **Real-time Animation** of the pendulum-cart system.
- **Noise Option** to simulate realistic measurement disturbances.
- **GUI** for easy interaction and parameter tuning.

---

## Requirements

- Python 3.7+
- `numpy`
- `matplotlib`
- `scipy`
- `tkinter` (usually included with Python)

To install required packages:

```bash
pip install numpy matplotlib scipy

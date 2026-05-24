# Doodler: Mechanical Spirograph Simulator

Doodler is a Python-based simulation of a mechanical drawing machine. It models two robotic arms (Motor A and Motor B) that interact to move a pen over a rotating platform (Motor T). The resulting intersection of these arms creates intricate, mathematical "spirograph" patterns.

## Features

- **Kinematic Simulation**: Models the geometry of a multi-link arm system.
- **Dynamic Motor Speeds**: Motors follow a configurable sinusoidal speed profile: `omega = c1 + c2 * sin(c3 * t + c4)`.
- **Dual-Frame Visualization**: 
  - **O Frame**: Shows the pen's path from a stationary observer's perspective.
  - **T Frame**: Shows the actual "doodle" as it appears on the rotating paper.
- **Geometric Validation**: Checks for physical feasibility (intersection existence) before and during simulation.

## Physics and Logic

The system consists of:
1.  **Primary Arms ($L_1$)**: Fixed at specific mounting points.
2.  **Secondary Arms ($L_2$)**: Rotating arms driven by motors at the end of the primary arms.
3.  **Tertiary Arms ($L_3$)**: Rods that connect the secondary arms to a common pen point.

The pen's position is calculated using a circle-circle intersection algorithm (`find_intersection`) based on the current positions of the ends of the secondary arms.

## Requirements

- Python 3.x
- NumPy
- Matplotlib

## Installation

Clone the repository and install the dependencies:

```bash
pip install numpy matplotlib
```

## Usage

Run the main simulation script:

```bash
python doodler.py
```

## Configuration

You can adjust the machine's geometry and behavior by modifying the constants at the top of `doodler.py`:

- **Arm Lengths**: `LA1`, `LA2`, `LA3`, etc.
- **Motor Parameters**: `Motor_A_params`, `Motor_B_params`, and `Motor_T_params`. These control the base speed, oscillation amplitude, and frequency of each motor.
- **Simulation Settings**: `t_final` (total duration) and `dt` (time step).

## Visualization

When the simulation completes successfully, two windows will appear:
1.  **Motor Angles Over Time**: A plot showing the angular displacement of all three motors.
2.  **Spirograph Plots**: Side-by-side comparison of the pen path in the stationary vs. rotating frames, including a boundary box representing the paper limits.
# IRB2400 Welding Trajectory Project

This project implements kinematic modeling, trajectory planning, and dynamic analysis for the ABB IRB2400/10 industrial robot.
The code is refactored into three Python modules using class-based encapsulation, while preserving all original functionality and results.

File Structure

irb2400Code/
│
├── irb2400_core.py
│   Robot kinematics and dynamics model (MDH, FK, IK, Newton–Euler)
│
├── irb2400_planning.py
│   Trajectory planning utilities (S-curve move, smoothing)
│
└── irb2400_main.py
    Main script: executes the full project workflow, plotting and animation

Implemented Tasks
- 	Modified DH modeling of ABB IRB2400/10
-	Forward and inverse kinematics
-	Tool–station coordinate transformation
-	Time-optimal joint-space motion from initial pose to welding start point
-	Circular welding trajectory in Cartesian space
-	Joint velocity, acceleration, and torque computation
-	Visualization of joint states and robot motion animation

How to Run

Place all three files in the same directory and run:

python irb2400_main.py

The script will:
-	Print numerical results for each task
-	Plot joint position, velocity, acceleration, and torque
-	Animate the robot motion and welding trajectory

Notes
-	This refactoring is purely structural: algorithms, parameters, and numerical behavior are unchanged from the optimized single-file version.
-	Dynamic parameters (mass, center of mass, inertia) are approximate and intended for feasibility analysis rather than exact torque matching.
-	Minor residual oscillations in acceleration/torque are due to numerical differentiation and simplified dynamics, and are expected in this modeling level.

---


5-DOF PRRRR Robotic Manipulator Project

Overview
--------
This project demonstrates the modeling, simulation, and control of a 5-DOF PRRRR robotic manipulator, featuring one prismatic and four revolute joints. The manipulator is capable of executing complex trajectories and poses through effective kinematic and dynamic modeling. MATLAB is used for solving forward and inverse kinematics, performing dynamic simulations, and generating motion trajectories.

Features
--------
- 5-DOF Robotic Manipulator (PRRRR): One prismatic joint and four revolute joints.
- Trajectory Planning: Smooth motion profiles are generated using spline interpolation.
- Inverse Kinematics: Analytical solution for the inverse kinematics problem.
- Dynamic Simulation: Simulate joint torques, velocities, and accelerations.
- PD Control: Proportional-Derivative control for accurate joint positioning.

Structure
---------
- Kinematics and Dynamics:
  - Analytical solutions for forward and inverse kinematics.
  - Dynamic equations derived using the Lagrange method, enabling torque and force calculations.

- Control Strategy:
  - A PD controller is implemented to minimize error and regulate joint positions during motion.

Results
-------
- Inverse Kinematics: Successfully computed for target positions and orientations.
- Trajectory Planning: Smooth trajectories were generated for various motion profiles.
- Simulation: The manipulator follows complex paths and handles dynamic constraints effectively.

How to Run
----------
1. Clone the repository.
2. Open the MATLAB environment.
3. Run the main script (`main_simulation.m`) to initialize the robot and start the simulation.
4. Adjust trajectory parameters in the script to test different configurations.

Demonstration
-------------
### Trajectory Visualization:
![Trajectories](./trajectories.png)

### Final Simulation:
![Simulation](./sim.gif)

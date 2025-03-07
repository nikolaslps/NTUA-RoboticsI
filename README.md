# Robotics-I Project (2024-25)

## Course Information
- **Course**: "Robotics I: Analysis, Control, Laboratory"
- **Academic Year**: 2024-25
- **Institution**: National Technical University of Athens (NTUA)
- **Department**: School of Electrical and Computer Engineering  
- **Topic**: Industrial Robotic Manipulator (KUKA)

## Project Overview
This project involves the theoretical analysis and kinematic simulation of a **6-DOF KUKA industrial robotic arm**. The key objectives include:
- **Denavit-Hartenberg (D-H) parameterization**
- **Forward and inverse kinematics**
- **Jacobian matrix derivation**
- **Singular configurations analysis**
- **Trajectory planning**
- **Kinematic simulation using Matlab**

## Tasks Breakdown

### Part A: Theoretical Analysis
1. **D-H Parameter Assignment**  
   - Define reference frames and derive the transformation matrix.
2. **Forward Kinematics**  
   - Compute the position and orientation of the end effector.
3. **Jacobian Matrix Calculation**  
   - Derive the differential kinematic model.
4. **Singular Configurations**  
   - Analyze critical configurations where the Jacobian loses rank.
5. **Inverse Kinematics**  
   - Solve for joint angles given a desired end-effector position.

### Part B: Kinematic Simulation
1. **Trajectory Design**  
   - Implement a smooth trajectory between two points, ensuring continuity in position and velocity.
2. **Simulation and Visualization**  
   - Compute joint trajectories {q1, q2, q3} and angular velocities.
   - Plot key variables over time.
   - Generate an animation of the robotic arm's motion.

---

### Notes
- **Assumptions**: For simplification, joints **q4, q5, and q6** are considered fixed at zero.
- **Adjustable Parameters**: Initial and final positions, time period, and workspace dimensions are flexible for simulation purposes.

---

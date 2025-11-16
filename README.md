# EE5112 Group6 - STOMP Path Planning Project

Our MATLAB project implements **STOMP (Stochastic Trajectory Optimization for Motion Planning)** for a frankaEmikaPanda robotic arm. It includes scripts for trajectory planning, collision avoidance, Boundary Constraints and visualization of robot motion in environments with obstacles.

---

## Project Structure

### File path
- `task1/` – ProgramCompletion and Collision-Free Path Planning.
- `task2/` – PathPlanning for a Different Manipulator.
- `task3/` – ForwardKinematics Based on PoE.
- `task4/` – Collision Avoidance Scenario Design.
- `task5/` – PathPlanningwithJointConstraints.
- `Total/` - Contains all the code for the final task. Run the project from this folder.

### Main Scripts
- `main.m` – Main script to run the entire project.
- `KINOVA_STOMP_Path_Planning.m` – Main STOMP path planning script.
- `RunLiveScript.m` – Utility to run the live script version.

### Robot and Environment Utilities
- `InitialVisualizer.m` – Initialize scene visualization with frankaEmikaPanda robot.
- `createCollisionBox.m` – Create obstacles in the environment.
- `helperSTOMP.m` – STOMP functions for trajectory optimization.
- `updateJointsWorldPosition.m` – Compute world positions of robot joints.

### STOMP Core Functions
- `stompDTheta.m` – STOMP delta-theta update.
- `stompSamples.m` – Generate trajectory samples.
- `stompTrajCost.m` – Compute trajectory cost.
- `stompObstacleCost.m` – Compute obstacle cost.
- `stompOrientationCost.m` – Compute end-effector orientation cost.
- `stompUpdateTheta.m` – Update trajectory based on samples.
- `stompUpdateProb.m` – Update probability of trajectories.
- `stompRobotSphere.m` – Define robot spheres for collision approximation.

### Collision & Distance Utilities
- `getTransformPoE.m` – Compute forward kinematics using the PoE (Product of Exponentials) method.
- `sEDT_3d.m` – Compute 3D Signed Euclidean Distance Transform for collision checking.

### Visualization
- `visual.m` – General visualization utilities.
- `/video` – Folder containing generated video outputs of robot motion.

### Testing
- `test.m` – Test scripts for various functions and utilities.

---

## How to Run

1. Open MATLAB and navigate to the project directory.
2. Run `main.m` to execute the full STOMP path planning workflow.
3. Optional: Use `RunLiveScript.m` to run the live script version `Path_Planning.mlx`.
4. Visualization of robot motion and obstacles will be displayed during execution.

---
## Effect Demonstration
![Robot Path Planning](Total/video/Output.gif)
---

## Dependencies

- MATLAB R2020a or later.
- Robotics System Toolbox.
- Image Processing Toolbox for visualization utilities.



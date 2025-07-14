# Dynamics-of-Tracked-Robot
Hi there! 👋

This plugin was born out of my passion for robotics and simulation. I wanted to understand how tracked vehicles behave under slip conditions and implement a control system that closely mirrors real-world dynamics This repository contains a custom **Gazebo plugin** for simulating the dynamics and control of a **tracked vehicle**. The plugin models complex interactions such as slip, contact forces, and track-terrain dynamics with high precision, enabling accurate simulation of tracked robot behavior in realistic environments.

## 🚀 Project Overview

Tracked vehicles are essential in scenarios requiring high traction and mobility over challenging terrains. This simulation framework bridges the gap between simplified kinematic models and full dynamic behavior, offering:

- Differential steering kinematics
- Track slip and terrain interaction modeling
- RK4-based dynamic integration
- PID-based linear and angular velocity control
- ROS-compatible command and odometry publishing

## 📁 Repository Structure

- `TrackedVehicle.cc` – Plugin source file implementing core dynamics and control logic  
- `TrackedVehicle.hh` – Header file for tracked vehicle parameters and class definitions  
- `README.md` – This documentation  
- `LICENSE` – Open-source license (Apache 2.0)

## ⚙️ Features

- ✅ Accurate modeling of:
  - Longitudinal and lateral slip
  - Contact and shear forces
  - Rolling resistance
- ✅ Realistic physics integration using Runge-Kutta 4th order method
- ✅ PID control for linear and angular motion
- ✅ Compatible with ROS for autonomous and manual control
- ✅ Odometry output for localization and navigation integration

## 🧪 Validated Scenarios

The plugin was tested across diverse scenarios:
- Step input acceleration
- U-turn, circular motion, and figure-eight paths
- Varying terrain friction and vehicle mass/inertia
- Manual and autonomous control inputs

Simulation results showed high accuracy with:
- < 0.05 m path deviation
- ±0.1 m/s velocity error
- Smooth settling within ~1.8 s

## 🛠 Dependencies

- [Gazebo Sim](https://gazebosim.org/)
- C++17
- ROS 2 (tested with Humble)
- SDF (Simulation Description Format)

## 🔮 Future Improvements

- Add terrain deformability and soil interaction
- Integrate model predictive control (MPC)
- Enable swarm simulation and inter-vehicle dynamics
- SLAM and autonomous navigation capabilities

## 📄 License

Licensed under the [Apache 2.0 License](LICENSE).

---

✍️ *Author Note*: This project was an incredible learning experience in physics-based simulation and control systems. If you found it helpful or have feedback, feel free to connect!

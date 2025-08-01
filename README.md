# LQR and TVLQR Control Demos in MATLAB

This repository contains a collection of control system simulations using **LQR (Linear Quadratic Regulator)** and **TVLQR (Time-Varying LQR)** controllers. These demos were originally built during Summer 2023 and have been reorganized and updated with cleaner visualizations and modular code.

---

## 📁 Project Structure


---

## 🚗 Demos

### 1. Cart-Pole LQR
- Balances an inverted pendulum on a cart using state-feedback LQR
- Simulated in MATLAB using linearized dynamics
- Includes visualization of pole angle and cart trajectory

### 2. Car Lane Keeping (LQR)
- Uses LQR to keep a car centered in a lane
- State vector: `[lateral error, heading error, ...]`
- Based on a simplified bicycle model

### 3. Car Lane Keeping (TVLQR)
- Same model as above, but tracks a time-varying reference path
- Recomputes LQR gain `K(t)` along trajectory

---

## 🧠 Key Concepts
- Linear Quadratic Regulator (LQR)
- Time-Varying LQR (TVLQR)
- Linearization of nonlinear dynamics
- MATLAB ODE solvers and simulation
- Visualization of state trajectories

---

## ▶️ How to Run

Each demo is a self-contained `.m` script that can be run in MATLAB:

```matlab
% Run cart-pole LQR demo
cd cartpole
lqr_cartpole

% Run car lane-keeping LQR demo
cd ../car_lane_keeping
lqr_lane

% Run car lane-keeping TVLQR demo
tvlqr_lane

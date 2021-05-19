# MPC project

## Introduction
In this Model Predictive Control (MPC) project, we implement a variety of controllers for temperature regulation of a vaccination center.

More details, check "instructions.pdf".

## How to start
1. git clone (or fork)
2. install MPT & Yalmip (follow instructions in "instructions.pdf")
3. run "run_simulation.m" in /src

## Workflow

1. Build a model and discretize it. Simulate it without any controller.
2. Unconstrained LQR controller for LTI systems.
3. Compute initial constraints for x(0)_LQR.
4. Basic MPC implementation.
5. MPC with theoretical closed loop guarantees (feasibility and stability) 
   (zero terminal; terminate in LQR feasible set).
6. Soft constraint MPC.
7. Offset MPC for constant unknown disturbance.
8. MPC and C compiler (with FORCES Pro).

## Figures and results

check /figs for figures.

check "MPC_report.pdf" for details.

## Archive

In /archive, lecture notes and summary of class named "Model Predictive Control" in ETH Zurich are provided for reference.


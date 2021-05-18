"""
This example demonstrates how to use the SQP_NLP real-time iteration solver to control a DC motor

The DC motor must track a reference on its angular speed.

Variables are collected stage-wise into z = [u; x(1); x(2)].

(c) Embotech AG, Zurich, Switzerland, 2013-2021.
"""

import numpy as np
import matplotlib.pyplot as plt
import forcespro.nlp
import casadi

def main():
    # populate model struct
    # Problem dimensions
    model = forcespro.nlp.SymbolicModel(20)  # horizon length
    model.nvar = 3  # number of variables
    model.neq = 2   # number of equality constraints
    model.nh = 0    # number of inequality constraint functions
    model.npar = 1  # number of run-time parameters (in this case the 1-dimensional reference we are tracking)

    # Objective function
    model.LSobjective = lambda z, p: np.sqrt(100) * (z[2] - p)
    model.LSobjectiveN = lambda z, p: np.sqrt(100) * (z[2] - p)

    # initial condition
    # xinit = np.zeros((model.neq, 1))
    model.xinitidx = range(1, model.nvar)

    # State and input bounds
    model.lb = np.array([1, -5, -10])
    model.ub = np.array([1.6, 5, 2.004])

    # dynamics
    model.continuous_dynamics = dynamics
    model.E = np.concatenate([np.zeros((model.neq, 1)), np.eye(model.neq)], axis=1)

    # set codeoptions
    codeoptions = forcespro.CodeOptions("FORCESNLPsolver")
    codeoptions.solvemethod = 'SQP_NLP'  # generate SQP-RTI solver
    codeoptions.BuildSimulinkBlock = 0
    codeoptions.nlp.integrator.type = 'ERK4'
    integration_step = 0.01
    codeoptions.nlp.integrator.Ts = integration_step
    codeoptions.nlp.integrator.nodes = 1
    codeoptions.nlp.hessian_approximation = 'gauss-newton'
    codeoptions.timing = 1
    codeoptions.server = 'https://forces.embotech.com'

    # generate FORCESPRO solver
    solver = model.generate_solver(codeoptions, static=True)

    # run simulation
    simLength = 500  # simulate 5 seconds

    # populate run time parameters struct
    params = {
        "all_parameters": np.tile(2, (model.N,)),
        "xinit": np.zeros((model.neq,)),  # initial condition to ODE
        "x0": np.tile([1.2, 0, 0], (model.N,)),  # initial guess
        "reinitialize": 0
    }
    x = params["xinit"]

    # collect simulation data
    time = np.zeros((simLength, 1))  # store time vector (simulation time)
    solveTime = np.zeros((simLength, 1))  # store solve time
    fevalsTime = np.zeros((simLength, 1))  # store function evaluations time
    qpTime = np.zeros((simLength, 1))  # store time it takes to solve quadratic approximation)
    obj = np.zeros((simLength, 1))  # store closed-loop objective objective value
    referenceValue = np.zeros((simLength, 1))  # store reference value which is tracked
    angularSpeed = np.zeros((simLength, 1))  # store angular speed

    for k in range(0, simLength):

        # Solve optimization problem
        output, exitflag, info = solver.solve(params)

        # check quality of output
        assert exitflag == 1, 'FORCESPROSolver failed to find good output'

        # extract control
        u = output["x01"][0:1]

        # integrate ODE
        p = params["all_parameters"][0]
        x = forcespro.nlp.integrators.integrate(dynamics, x, u, p,
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integration_step, steps=codeoptions.nlp.integrator.nodes).full().reshape(2,)

        # prepare params struct for next simulation step
        if k == 250:
           params["all_parameters"] = np.tile(-2, (model.N,))
           # change reference half way through simulation to test robustness of controller
        params["xinit"] = x

        # collect simulation data
        time[k] = k*integration_step
        solveTime[k] = info.solvetime
        fevalsTime[k] = info.fevalstime
        qpTime[k] = info.QPtime
        obj[k] = 0.5 * (model.LSobjective(np.concatenate([u, x]), p) * model.LSobjective(np.concatenate([u, x]), p))
        referenceValue[k] = p
        angularSpeed[k] = x[1]

    # Plot simulation results

    # Solve time
    plt.figure('solvetime vs time')
    plt.plot(time, solveTime, 'b', linewidth=2)
    plt.xlabel('Simulation time (s)')
    plt.ylabel('solvetime (s)')
    plt.grid('both')

    # function evaluations time
    plt.figure('fevalstime vs time')
    plt.plot(time, fevalsTime, 'b', linewidth=2)
    plt.xlabel('Simulation time (s)')
    plt.ylabel('fevalstime (s)')
    plt.grid('both')

    # QP time
    plt.figure('QPtime vs time')
    plt.plot(time, qpTime, 'b', linewidth=2)
    plt.xlabel('Simulation time (s)')
    plt.ylabel('QPtime (s)')
    plt.grid('both')

    # Closed-loop objective
    plt.figure('Closed-loop objective value vs time')
    plt.plot(time, obj, 'b', linewidth=2)
    plt.xlabel('Simulation time (s)')
    plt.ylabel('Closed-loop objective value')
    plt.grid('both')

    # Distance to reference
    plt.figure('Angular speed and reference')
    plt.plot(time, angularSpeed, 'b', linewidth=2)
    plt.plot(time, referenceValue, 'r', linewidth=2)
    plt.legend(['Angular speed', 'reference value'])
    plt.xlabel('Simulation time (s)')

    plt.show()


def dynamics(x, u, p):
    ## model parameters
    # Armature inductance (H)
    La = 0.307
    # Armature resistance (Ohms)
    Ra = 12.548
    # Motor constant (Nm/A^2)
    km = 0.23576
    # Total moment of inertia (Nm.sec^2)
    J = 0.00385
    # Total viscous damping (Nm.sec)
    B = 0.00783
    # Load torque (Nm)
    tauL = 1.47
    # Armature voltage (V)
    ua = 60

    dx = casadi.vertcat((-1.0/La)*(Ra*x[0] + km*x[1]*u[0] - ua),
                        (-1.0/J)*(B*x[1] - km*x[0]*u[0] + tauL))

    return dx


if __name__ == "__main__":
    main()

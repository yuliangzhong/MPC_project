# F8 Crusader aircraft
# The control objective is to drive the angle of attack to zero by changing
# the tail deflection angle, which can be fully up or down (discrete control input).
#
#  min   xN'*P*xN + sum_{i=1}^{N-1} ui'*R*ui
# xi,ui
#       s.t. x1 = x
#            x_i+1 = f(x_i)       for i = 1...N-1 (aircraft dynamics)
#            xmin <= xi <= xmax   for i = 1...N
#            umin <= ui <= umax   for i = 1...N-1
#            u_i in {umin, umax}  for i = 1...N-1 (integer input)
#
# (c) Embotech AG, Zurich, Switzerland, 2020-2021

import numpy as np
import matplotlib.pyplot as plt
import forcespro
import forcespro.nlp
import casadi 

# Whether or not to provide an guess for the incumbent
guessIncumbent = True

solverName = 'F8aircraft_solver'
Nstages = 100

dimU = 1  # number of inputs
dimX = 3  # number of states
dimZ = dimU + dimX  # number of stage variables

# Model with stage variable (u', x')'
model = forcespro.nlp.SymbolicModel(Nstages)
model.nvar = dimZ
model.neq = dimX

# Boundary conditions
model.xinitidx = range(dimU, dimZ)

# Bounds
# In the first stage, we have parametric bounds on the inputs.
model.lbidx[0] = range(0, dimU)
model.ubidx[0] = range(0, dimU)
# In the following stages, all stage variables (inputs and states) are bounded.
for i in range(1, Nstages):
    model.lbidx[i] = range(0, dimZ)
    model.ubidx[i] = range(0, dimZ)

# Dynamics
wa = 0.05236
wa2 = wa**2
wa3 = wa**3
continuous_dynamics = lambda x, u: casadi.vertcat(-0.877 * x[0] + x[2] - 0.088 * x[0] * x[2] + 0.47 * x[0] * x[0] - 0.019 * x[1] * x[1] - x[0] * x[0] * x[2]
                                                + 3.846 * x[0] * x[0] * x[0] - 0.215 * wa * (2 * u[0] - 1) + 0.28 * x[0] * x[0] * wa * (2 * u[0] - 1) + 0.47 * x[0] * wa2 * (2 * u[0] - 1) * (2 * u[0] - 1)
                                                + 0.63 * wa3 * (2 * u[0] - 1) * (2 * u[0] - 1) * ( 2 * u[0] - 1),
                                                x[2],
                                                -4.208 * x[0] - 0.396 * x[2] - 0.47 * x[0] * x[0] - 3.564 * x[0] * x[0] * x[0] - 20.967 * wa * (2 * u[0] - 1) + 6.265 * x[0] * x[0] * wa * (2 * u[0] - 1)
                                                + 46.0 * x[0] * wa2 * (2 * u[0] - 1) * (2 * u[0] - 1) + 61.4 * wa3 * (2 * u[0] - 1) * (2 * u[0] - 1) * (2 * u[0] - 1))
model.continuous_dynamics = continuous_dynamics
model.E = np.concatenate([np.zeros((dimX, dimU)), np.eye(dimX)], axis=1)

# Objective
model.objective =  lambda z: 0.0001 * z[0]**2
model.objectiveN = lambda z: 150 * z[1]**2 + 5 * z[2]**2 + 5 * z[3]**2

# Integer indices
model.intidx = [0]

# Define outputs
outputs = [('TailAngle', range(0, model.N), 0),
           ('AngleAttack', range(0, model.N), 1),
           ('PitchAngle', range(0, model.N), 2),
           ('PitchAngleRate', range(0, model.N), 3)]

# Set code-generation options
codeoptions = forcespro.CodeOptions(solverName)
codeoptions.printlevel = 0
codeoptions.maxit = 2000
codeoptions.timing = 0
codeoptions.nlp.integrator.type = 'IRK2'
codeoptions.nlp.integrator.Ts = 0.05
codeoptions.nlp.integrator.nodes = 20
# Specify maximum number of threads to parallelize minlp search
codeoptions.minlp.max_num_threads = 8
codeoptions.BuildSimulinkBlock = 0
codeoptions.nlp.ad_tool = 'casadi-3.5.1'

# Options for providing incumbent guess at runtime
if guessIncumbent:
    codeoptions.minlp.int_guess = 1
    codeoptions.minlp.round_root = 0
    codeoptions.minlp.int_guess_stage_vars = [1]

# Generate MINLP solver
solver = model.generate_solver(codeoptions, outputs)

# Set run-time parameters
problem = {}
problem["lb{:03d}".format(1)] = [0]
problem["ub{:03d}".format(1)] = [1]
for s in range(1, Nstages-1):
    problem["lb{:03d}".format(s+1)] = np.concatenate([[0], -1e1 * np.ones(dimX)])
    problem["ub{:03d}".format(s+1)] = np.concatenate([[1], 1e1 * np.ones(dimX)])
problem["lb{:03d}".format(Nstages)] = np.concatenate([[0], -1e1 * np.ones(dimX)])
problem["ub{:03d}".format(Nstages)] = np.concatenate([[1], 1e1 * np.ones(dimX)])

problem["x0"] = np.tile(np.zeros((dimZ, 1)), (Nstages, 1))
problem["xinit"] = np.zeros((dimX, 1))
problem["xinit"][0] = 0.4655
# FORCESPRO integer search will run on 2 thread
problem["parallelStrategy"] = 0 # Default value
problem["numThreadsBnB"] = 2

# Integer guess
if guessIncumbent:
    for s in range(0, Nstages):
        problem["int_guess{:03d}".format(s+1)] = [0]
    for s in range(0, 2):
        problem["int_guess{:03d}".format(s+1)] = [1]
    problem["int_guess{:03d}".format(39)] = [1]
    for s in range(40, 42):
        problem["int_guess{:03d}".format(s+1)] = [1]
    for s in range(84, 90):
        problem["int_guess{:03d}".format(s+1)] = [1]

# Call MINLP solver
sol, exitflag, info = solver.solve(problem)

# plot
time = np.arange(0, 4.96, codeoptions.nlp.integrator.Ts)

plt.step(time, 0.05236 * (2 * sol["TailAngle"] - 1))
plt.grid('both')
plt.title('Tail deflection angle (rad)')
plt.tight_layout()

plt.figure()
plt.subplot(3, 1, 1)
plt.grid('both')
plt.title('Angle of attack (rad)')
plt.plot(time, sol["AngleAttack"], 'r')
plt.subplot(3, 1, 2)
plt.grid('both')
plt.title('Pitch angle (rad)')
plt.plot(time, sol["PitchAngle"], 'b')
plt.subplot(3, 1, 3)
plt.grid('both')
plt.title('Pitch angle rate (rad/sec)')
plt.plot(time, sol["PitchAngleRate"], 'b')
plt.tight_layout()
plt.show()


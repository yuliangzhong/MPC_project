import numpy as np
import matplotlib.pyplot as plt
import forcespro
import forcespro.nlp
import casadi 

"""
Simple MPC - double integrator example for use with FORCESPRO
with rate constraints on du

 min   xN'*P*xN + sum_{i=1}^{N-1} xi'*Q*xi + ui'*R*ui
xi,ui
     s.t. x1 = x
       x_i+1 = A*xi + B*ui  for i = 1...N-1
       xmin  <= xi <= xmax   for i = 1...N
       umin  <= ui <= umax   for i = 1...N
       dumin <= u{i+1} - ui <= dumax for i= 1...N-1

 and P is solution of Ricatti eqn. from LQR problem

(c) Embotech AG, Zurich, Switzerland, 2013-2021
"""


# Model Definition
# ----------------

# system
A = np.array([[1.1, 1], [0, 1]])
B = np.array([[1], [0.5]])
nx, nu = np.shape(B)

# MPC setup
N = 10
Q = np.eye(nx)
R = np.eye(nu)
P = 10*Q
umin = -0.5
umax = 0.5
absrate = 0.05
dumin = -absrate
dumax = absrate
xmin = np.array([-5, -5])
xmax = np.array([5, 5])

# FORCESPRO multistage form
# assume variable ordering zi = [u{i+1}-ui; ui; xi] for i=1...N

# dimensions
model = forcespro.nlp.ConvexSymbolicModel(11)   # horizon length
model.nvar = 4   # number of variables
model.neq = 3   # number of equality constraints

# objective
model.objective = (lambda z: z[1]*R*z[1] +
                   casadi.horzcat(z[2], z[3]) @ Q @ casadi.vertcat(z[2], z[3]))
model.objectiveN = (lambda z: z[0]*R*z[0] +
                    casadi.horzcat(z[2], z[3]) @ P @ casadi.vertcat(z[2], z[3]))

# equalities
model.eq = lambda z: casadi.vertcat( z[0] + z[1],
                               casadi.dot(A[0, :], casadi.vertcat(z[2], z[3])) + B[0, :]*z[1],
                               casadi.dot(A[1, :], casadi.vertcat(z[2], z[3])) + B[1, :]*z[1])
              
model.E = np.concatenate([np.zeros((3, 1)), np.eye(3)], axis=1)

# initial state
model.xinitidx = [2, 3]

# inequalities
model.lb = np.concatenate([[dumin, umin], xmin])
model.ub = np.concatenate([[dumax, umax], xmax])


# Generate FORCESPRO solver
# -------------------------

# set options
options = forcespro.CodeOptions()
options.solvemethod = "PDIP_NLP"
options.printlevel = 0
options.overwrite = 1
options.nlp.bfgs_init = None

# generate code
solver = model.generate_solver(options)

# Run simulation
# --------------

x1 = [-4, 2]
kmax = 30
x = np.zeros((2, kmax + 1))
x[:, 0] = x1
u = np.zeros((1, kmax))
du = np.zeros((1, kmax))
problem = {}

solvetime = []
iters = []

for k in range(kmax):
    problem["xinit"] = x[:, k]

    # call the solver
    solverout, exitflag, info = solver.solve(problem)
    assert exitflag == 1, "Some problem in solver"

    du[:, k] = solverout["x01"][0]
    u[:, k] = solverout["x01"][1]
    solvetime.append(info.solvetime)
    iters.append(info.it)
    print(model.eq(solverout["x01"]))
    update = model.eq(np.concatenate([du[:,k], u[:, k], x[:, k]])).full().reshape(3,)
    x[:, k + 1] = update[1:]
    if k+1 < kmax:
      u[:, k+1] = update[0]


# Plot results
# ------------

fig = plt.gcf()
plt.subplot(3,1,1)
plt.grid('both')
plt.title('states')
plt.plot([1, kmax], [5, 5], 'r--')
plt.plot([1, kmax], [-5, -5], 'r--')
plt.ylim(1.1*np.array([-5, 5]))
plt.step(range(1, kmax+1), x[0, range(1, kmax + 1)])
plt.step(range(1, kmax+1), x[1, range(1, kmax + 1)])

plt.subplot(3,1,2)
plt.grid('both')
plt.title('input')
plt.plot([1, kmax], [0.5, 0.5], 'r--')
plt.plot([1, kmax], [-0.5, -0.5], 'r--')
plt.ylim(1.1*np.array([-0.5, 0.5]))
plt.step(range(1, kmax+1), u[0, range(0, kmax)])

plt.subplot(3,1,3)
plt.grid('both')
plt.title('input')
plt.plot([1, kmax], [absrate, absrate], 'r--')
plt.plot([1, kmax], [-absrate, -absrate], 'r--')
plt.ylim(1.1*np.array([-absrate, absrate]))
plt.step(range(1, kmax+1), du[0, range(0, kmax)])

plt.show()

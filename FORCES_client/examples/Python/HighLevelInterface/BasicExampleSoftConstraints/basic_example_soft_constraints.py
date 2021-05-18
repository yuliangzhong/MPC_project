import numpy as np
import matplotlib.pyplot as plt
import forcespro
import forcespro.nlp
import casadi 

"""
Simple MPC - double integrator example for use with FORCESPRO
with soft inequality constraints on u

 min   xN'*P*xN + sum_{i=1}^{N-1} xi'*Q*xi + ui'*R*ui
xi,ui
     s.t. x1 = x
       x_i+1 = A*xi + B*ui  for i = 1...N-1
       xmin <= xi <= xmax   for i = 1...N
       umin <= ui <= umax   for i = 1...N

 and P is solution of Ricatti eqn. from LQR problem

(c) Embotech AG, Zurich, Switzerland, 2013-2021
"""


# Model Definition
# ----------------

# system
A = np.array([[1.1, 1], [0, 1]])
B = np.array([[1], [0.5]])
nx, nu = np.shape(B)
lam = 8 # measure for penalty term

# MPC setup
N = 10
Q = np.eye(nx)
R = np.eye(nu)
P = 10*Q
umin = -0.5
umax = 0.5
xmin = np.array([-5, -5])
xmax = np.array([5, 5])

# FORCESPRO multistage form
# assume variable ordering zi = [si; ui; xi] for i=1...N

# dimensions
model = forcespro.nlp.SymbolicModel(11)   # horizon length
model.nvar = 4    # number of variables
model.neq = 2   # number of equality constraints
model.nh = 2

# objective with penalty term
model.objective = (lambda z: z[1]*R*z[1] + lam*z[0] +
                   casadi.horzcat(z[2], z[3]) @ Q @ casadi.vertcat(z[2], z[3]))
model.objectiveN = (lambda z: z[1]*R*z[1] + lam*z[0] *
                    casadi.horzcat(z[2], z[3]) @ P @ casadi.vertcat(z[2], z[3]))

# equalities
model.eq = lambda z: casadi.vertcat(casadi.dot(A[0, :], casadi.vertcat(z[2], z[3])) + B[0, :]*z[1],
                               casadi.dot(A[1, :], casadi.vertcat(z[2], z[3])) + B[1, :]*z[1])
              
model.E = np.concatenate([np.zeros((2, 2)), np.eye(2)], axis=1)

# initial state
model.xinitidx = [2, 3]

# relaxed inequalities
model.ineq = lambda z: casadi.vertcat( z[1] - z[0],
                                       z[1] + z[0])
model.hu = np.array([umax, +float('inf')])
model.hl = np.array([-float('inf'), umin])

# inequalities
model.lb = np.concatenate([[0, -float('inf')], xmin])
model.ub = np.concatenate([[float('inf'), float('inf')], xmax])


# Generate FORCESPRO solver
# -------------------------

# set options
options = forcespro.CodeOptions()
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
s = np.zeros((1, kmax))
problem = {}

solvetime = []
iters = []

for k in range(kmax):
    problem["xinit"] = x[:, k]

    # call the solver
    solverout, exitflag, info = solver.solve(problem)
    assert exitflag >= 0, "Some problem in solver"

    s[:, k] = solverout["x01"][0]
    u[:, k] = solverout["x01"][1]
    solvetime.append(info.solvetime)
    iters.append(info.it)

    x[:, k + 1] = model.eq(np.concatenate([s[:, k], u[:, k], x[:, k]])).full().reshape(2,)


# Plot results
# ------------

fig = plt.gcf()
plt.subplot(2,1,1)
plt.grid('both')
plt.title('states')
plt.plot([1, kmax], [5, 5], 'r--')
plt.plot([1, kmax], [-5, -5], 'r--')
plt.ylim(1.1*np.array([-5, 5]))
plt.step(range(1, kmax+1), x[0, range(1, kmax + 1)])
plt.step(range(1, kmax+1), x[1, range(1, kmax + 1)])

plt.subplot(2,1,2)
plt.grid('both')
plt.title('input')
plt.plot([1, kmax], [0.5, 0.5], 'r--')
plt.plot([1, kmax], [-0.5, -0.5], 'r--')
plt.ylim(1.1*np.array([-1, 1]))
plt.step(range(1, kmax+1), u[0, range(0, kmax)])

plt.show()

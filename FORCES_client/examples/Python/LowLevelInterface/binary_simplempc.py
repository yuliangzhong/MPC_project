import sys
from forcespro import *
import matplotlib.pyplot as plt

# Simple MPC - double integrator example for use with FORCESPRO
# 
# We have inputs that can take only two values in this example
#
#  min   xN'*P*xN + sum_{i=1}^{N-1} xi'*Q*xi + ui'*R*ui
# xi,ui
#       s.t. x1 = x
#            x_i+1 = A*xi + B*ui  for i = 1...N-1
#            xmin <= xi <= xmax   for i = 1...N
#            umin <= ui <= umax   for i = 1...N
#            ui \in {umin, umax} (ui is binary)
#
# and P is solution of Ricatti eqn. from LQR problem
#
# (c) embotech AG, Zurich, Switzerland, 2015.

# system
A = np.array([[1.1, 1.0], [0.0, 1.0]])
B = np.array([[1.0, 0.0], [0.5, -0.5]])

nx = 2
nu = 2

# MPC setup
N = 5
Q = 10 * np.eye(nx)
R = np.eye(nu)
P = np.array([[11.832254382324539,  0.247583610559956],
			  [ 0.247583610559956, 11.742272320044430]])
umin = np.array([-0.5])
umax = np.array([0.5])
xmin = np.array([-5, -5])
xmax = np.array([5, 5])

# FORCESPRO multistage form
# assume variable ordering zi = [ui, xi+1] for i=1...N-1
stages = MultistageProblem( N )
for i in range(N):
	# dimension
	stages.dims[i]['n'] = nx + nu 	# number of stage variables
	stages.dims[i]['r'] = nx		# number of equality constraints
	stages.dims[i]['l'] = nx + nu 	# number of lower bounds
	stages.dims[i]['u'] = nx + nu 	# number of upper bounds
	stages.bidx[i] = np.array([1, 2])      # which indices are binary?

	# cost
	if i == N-1:
		stages.cost[i]['H'] = np.vstack((np.hstack((R,np.zeros((nu,nx)))),np.hstack((np.zeros((nx,nu)),P))))
	else:
		stages.cost[i]['H'] = np.vstack((np.hstack((R,np.zeros((nu,nx)))),np.hstack((np.zeros((nx,nu)),Q))))
	stages.cost[i]['f'] = np.concatenate((-R.dot(np.ones((nu,1))), np.zeros((nx,1))))

	# lower bounds
	stages.ineq[i]['b']['lbidx'] = range(1, nu+nx+1) 		# lower bound acts on these indices
	stages.newParam('lb{:02d}'.format(i+1), [i+1], 'ineq.b.lb')

	# upper bounds
	stages.ineq[i]['b']['ubidx'] = range(1, nu+nx+1) 		# upper bound acts on these indices
	stages.newParam('ub{:02d}'.format(i+1), [i+1], 'ineq.b.ub')
		
	# equality constraints
	if i < N-1:
		stages.eq[i]['C'] = np.hstack((np.zeros((nx,nu)), A))
	if i > 0:
		stages.eq[i]['c'] = 0.5*B.dot(np.ones((nu,1)))
	stages.eq[i]['D'] = np.hstack((B, -np.eye(nx)))
	
	
stages.newParam('minusA_times_x0', [1], 'eq.c') # RHS of first eq. constr. is a parameter: z1=-A*x0
		
# define output of the solver
stages.newOutput('u0', 1, range(1,nu+1))

# solver settings
stages.codeoptions['name'] = 'MIQP_MPC'

# generate code
import get_userid
stages.generateCode(get_userid.userid)

# simulate
import MIQP_MPC_py
problem = MIQP_MPC_py.MIQP_MPC_params
x1 = np.array([-5, 3])
kmax = 30
X = np.zeros((nx,kmax+1))
X[:, 0] = x1
U = np.zeros((nu,kmax))
D = np.zeros((nu,kmax))

# set upper and lower bounds
for i in range(0, N):
	problem['lb{:02d}'.format(i+1)] = np.hstack((np.array([0.0, 0.0]), xmin))
	problem['ub{:02d}'.format(i+1)] = np.hstack((np.array([1.0, 1.0]), xmax))

for k in range(0, kmax):
	problem['minusA_times_x0'] = (-A.dot(X[:,k]) + 0.5*B.dot(np.ones((nu,1))).T)
	[solverout,exitflag,info] = MIQP_MPC_py.MIQP_MPC_solve(problem)
	if exitflag == 1:
		D[:,k] = solverout['u0']
		U[:,k] = D[:,k] - 0.5*np.ones((1,nu))
		print(info.solvetime)
	else:
		print(info)
		raise RuntimeError('Some problem in solver')
	print("Solver iteration {} finished".format(k))

	X[:,k+1] = np.dot(A,X[:,k]) + B.dot(U[:,k])

# plot
fig1 = plt.figure()
plt.subplot(2,1,1)
plt.axhline(y=max(xmax),c="red",zorder=0)
plt.axhline(y=max(xmin),c="red",zorder=0)
plt.step(range(0,kmax),X[0,0:kmax],where='post')
plt.step(range(0,kmax),X[1,0:kmax],where='post')
plt.title('states')
plt.xlim(0,kmax)
plt.ylim(1.1*min(xmin),1.1*max(xmax))
plt.grid()

plt.subplot(2,1,2)
plt.axhline(y=umax,c="red",zorder=0)
plt.axhline(y=umin,c="red",zorder=0)
plt.step(range(0,kmax),U[0,0:kmax],where='post')
plt.step(range(0,kmax),U[1,0:kmax],where='post')
plt.title('inputs')
plt.xlim(0,kmax)
plt.ylim(1.1*min(umin),1.1*max(umax))
plt.grid()

plt.show()


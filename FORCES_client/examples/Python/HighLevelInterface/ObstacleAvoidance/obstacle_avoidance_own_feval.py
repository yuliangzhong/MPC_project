# Example script for getting started with FORCESPRO NLP solver.

# --------------------------------------------------------------------------
# NOTE: This example shows how to pass C functions implementing the
# function evaluations to FORCES. There is an automated way of creating
# these C functions, which is explained in the other example file named
# "obstacle_avoidance.py"
# --------------------------------------------------------------------------

# This example solves an optimization problem for a car with the simple
# continuous-time, nonlinear dynamics (bicycle model):

#    dxPos/dt  = v*cos(theta + beta)
#    dyPos/dt  = v*sin(theta + beta)
#    dv/dt     = F/m
#    dtheta/dt = v/l_r*sin(beta)
#    ddelta/dt = phi

#    with:
#    beta      = arctan(l_r/(l_f + l_r)*tan(delta))

# where xPos,yPos are the position, v the velocity in heading angle theta 
# of the car, and delta is the steering angle relative to the heading 
# angle. The inputs are acceleration force F and steering rate phi. The 
# physical constants m, l_r and l_f denote the car's mass and the distance 
# from the car's center of gravity to the rear wheels and the front wheels.

# The car starts from standstill with a certain heading angle, and the
# optimization problem is to move the car as close as possible to a certain
# end position while staying inside a nonconvex area. In addition, an 
# elliptic obstacle must be avoided. Using an interactive window, 
# the position of the obstacle can be changed by the user and the car's 
# trajectory is adapted automatically.

# Quadratic costs for the acceleration force and steering are added to the
# objective to avoid excessive maneouvers.

# There are bounds on all variables except theta.

# Variables are collected stage-wise into 

#     z = [F phi xPos yPos v theta delta].

# See also FORCES_NLP

# (c) Embotech AG, Zurich, Switzerland, 2013-2021.


import sys
import numpy as np
import casadi
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec

def ellipse(bbox, ax=None,*args, **kwargs):
    """Draw an ellipse filling the given bounding box."""
    if ax==None:
        ax = plt.gca()
    x, y, w, h = bbox
    shape = matplotlib.patches.Ellipse((x + w/2.0, y + h/2.0), w, h, *args, **kwargs)
    ax.add_artist(shape)
    return ax

def generate_pathplanner():
    """
    Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function.
    """
    # Model Definition
    # ----------------

    # Problem dimensions
    N = 50 # horizon length
    model = forcespro.nlp.ExternalFunctionModel(N)
    model.nvar = 7  # number of variables
    model.neq = 5  # number of equality constraints
    model.nh = 2  # number of inequality constraint functions
    model.npar = 2 # number of runtime parameters

    # Define source file containing function evaluation code
    model.set_main_callback("C/myfevals.c", function="myfevals")
    model.add_auxiliary("C/car_dynamics.c")

    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = np.concatenate([np.zeros((5,2)), np.eye(5)], axis=1)

    # Inequality constraints
    # Simple bounds
    #  upper/lower variable bounds lb <= z <= ub
    #                     inputs                 |  states
    #                     F          phi            x    y     v    theta         delta
    model.lb = np.array([-5.,  np.deg2rad(-40.),  -3.,   0.,   0,   -np.inf, -0.48*np.pi])
    model.ub = np.array([+5.,  np.deg2rad(+40.),   0.,   3.,   2.,   np.inf,  0.48*np.pi])

    # Upper/lower bounds for inequalities hl <= h(z,p) <= hu
    model.hu = np.array([9, +np.inf])
    model.hl = np.array([1, 0.7**2])

    # Initial condition on vehicle states x
    model.xinitidx = range(2,7) # use this to specify on which variables initial conditions
    # are imposed

    # Solver generation
    # -----------------

    # Set solver options
    codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    codeoptions.maxit = 400     # Maximum number of iterations
    codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but 
    #                             not for timings)
    codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 
    #                             2 optimize for speed, 3 optimize for size & speed
    codeoptions.nlp.bfgs_init = 3.0*np.identity(7) # initialization of the hessian
    #                             approximation
    codeoptions.noVariableElimination = 1.
    # change this to your server or leave uncommented for using the 
    # standard embotech server at https://forces.embotech.com 
    # codeoptions.server = 'https://forces.embotech.com'
    
    # Creates code for symbolic model formulation given above, then contacts 
    # server to generate new solver
    solver = model.generate_solver(options=codeoptions)

    return model,solver

def calculate_path(event,model,solver,problem):
    """
    This callback is executed on every click inside the plot and estimates the
    position of the click from noisy measurements by calling the previously
    generated solver.
    """
    params = np.array([event.xdata,event.ydata])

    # make sure user clicked valid position
    if (params[0] == None) or (params[1] == None):
        sys.stderr.write("The obstacle position you clicked is out of bounds.\n")
    elif (params[0] < model.lb[2]) or (params[0] > model.ub[2]) \
        or (params[1] < model.lb[3]) or (params[1] > model.ub[3]):
        sys.stderr.write("The obstacle position you clicked is out of bounds.\n")
    elif ((params[0] - problem["xinit"][0])**2 + (params[1] - \
        problem["xinit"][1])**2 < model.hl[1]):
        sys.stderr.write("The initial position cannot lie within obstacle.\n")
    else:       
        print('You clicked X: {}, Y: {}'.format(params[0], params[1])) 
        # Simulation
        # ----------       
        # Set runtime parameters
        problem["all_parameters"] = np.transpose(np.tile(params,(1,model.N)))

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)

        # Make sure the solver has exited properly.
        if exitflag < 0:
            sys.stderr.write("Some error in FORCESPRO solver. exitflag={}\n"\
                .format(exitflag))
        elif exitflag == 0:
            sys.stderr.write(("Error in FORCESPRO solver: Maximum number of iterations was reached.\n"))
        else:
            sys.stderr.write(("FORCESPRO took {} iterations and {} seconds to "
            "solve the problem.\n".format(info.it, info.solvetime)))

            # Update plots
            # ------------
            # extract output of solver
            temp = np.zeros((np.max(model.nvar), model.N))
            for i in range(0, model.N):
                temp[:, i] = output['x{0:02d}'.format(i+1)]
            u = temp[0:2, :]
            x = temp[2:7, :]

            updatePlots(x,u,model,params)

def updatePlots(x,u,model,params):
    """Deletes old data sets in the current plot and adds the new data sets 
    given by the arguments x, u and params to the plot.
    x: matrix consisting of a set of state column vectors
    u: matrix consisting of a set of input column vectors
    params: position of obstacle
    model: model struct required for the code generation of FORCESPRO
    """

    fig = plt.gcf()
    ax_list = fig.axes

    # delete old results
    ax_list[0].get_children().pop(4).remove() # remove obstacle at old position
    ax_list[0].get_lines().pop(-1).remove()   # remove old trajectory
    ax_list[1].get_lines().pop(-1).remove()   # remove old velocity
    ax_list[2].get_lines().pop(-1).remove()   # remove old heading angle
    ax_list[3].get_lines().pop(-1).remove()   # remove old steering angle
    ax_list[4].get_lines().pop(-1).remove()   # remove old acceleration force
    ax_list[5].get_lines().pop(-1).remove()   # remove old steering rate
    
    # plot new results
    ax_list[0].plot(x[0,:], x[1,:], 'b-')           # plot new trajectory
    ellipse((params[0]-np.sqrt(model.hl[1]), \
        params[1]-np.sqrt(model.hl[1]), \
        2*np.sqrt(model.hl[1]), \
        2*np.sqrt(model.hl[1])), \
        ax=ax_list[0], fill=False, linestyle=":")   # plot obstacle at new position
    ax_list[1].plot(x[2,:],'b-')                    # plot new velocity
    ax_list[2].plot(np.rad2deg(x[3, :]),'b-')       # plot new heading angle
    ax_list[3].plot(np.rad2deg(x[4, :]),'b-')       # plot new steering angle
    ax_list[4].step(range(0, model.N), u[0, :],'b-')# plot new acceleration force
    ax_list[5].step(range(0, model.N), \
        np.rad2deg(u[1, :]),'b-')                   # plot new steering rate

    plt.show()

def plotAndMakeInteractive(x,u,model,params,problem,solver):
    """Creates a plot, adds the data sets provided by the arguments x, u, and
    params to the plot and makes the plot interactive by connecting a callback function
    x: matrix consisting of a set of state column vectors
    u: matrix consisting of a set of input column vectors
    params: position of obstacle
    model: model struct required for the code generation of FORCESPRO
    """
    fig = plt.figure()
    plt.clf()
    gs = GridSpec(5,2,figure=fig)

    # plot trajectory
    fig.add_subplot(gs[:,0])
    ellipse((-np.sqrt(model.hl[0]), -np.sqrt(model.hl[0]), 2*np.sqrt(model.hl[0]), \
        2*np.sqrt(model.hl[0])), fill=False, linestyle=":")
    ellipse((-np.sqrt(model.hu[0]), -np.sqrt(model.hu[0]), 2*np.sqrt(model.hu[0]), \
        2*np.sqrt(model.hu[0])), fill=False, linestyle=":")
    l1, = plt.plot(problem["xinit"][0], problem["xinit"][1], 'bx')
    plt.title('Click into figure to place obstacle')
    #plt.axis('equal')
    plt.xlim([-3, 0])
    plt.ylim([0, 3])
    plt.xlabel('x-coordinate')
    plt.ylabel('y-coordinate')
    l2, = plt.plot(x[0,:],x[1,:],'b-')
    ellipse((params[0]-np.sqrt(model.hl[1]), params[1]-np.sqrt(model.hl[1]), \
        2*np.sqrt(model.hl[1]), 2*np.sqrt(model.hl[1])), fill=False, linestyle=":")
    plt.legend((l1,l2),('init pos','trajectory'),loc='upper left')
    fig.canvas.mpl_connect('button_press_event', \
        lambda event: calculate_path(event,model,solver,problem))

    # plot velocity
    fig.add_subplot(5,2,2)
    plt.grid("both")
    plt.title('Velocity')
    plt.plot([0, model.N], np.transpose([model.ub[4], model.ub[4]]), 'r:')
    plt.plot([0, model.N], np.transpose([model.lb[4], model.lb[4]]), 'r:')
    plt.plot(x[2, :],'b-')

    # plot heading angle
    fig.add_subplot(5,2,4)
    plt.grid("both")
    plt.title('Heading angle')
    plt.ylim([0, 180])
    plt.plot([0, model.N], np.rad2deg(np.transpose([model.ub[5], model.ub[5]])), 'r:')
    plt.plot([0, model.N], np.rad2deg(np.transpose([model.lb[5], model.lb[5]])), 'r:')
    plt.plot(np.rad2deg(x[3, :]),'b-')

    # plot steering angle
    fig.add_subplot(5,2,6)
    plt.grid("both")
    plt.title('Steering angle')
    plt.plot([0, model.N], np.rad2deg(np.transpose([model.ub[6], model.ub[6]])), 'r:')
    plt.plot([0, model.N], np.rad2deg(np.transpose([model.lb[6], model.lb[6]])), 'r:')
    plt.plot(np.rad2deg(x[4, :]),'b-')

    # plot acceleration force
    fig.add_subplot(5,2,8)
    plt.grid("both")
    plt.title('Acceleration force')
    plt.plot([0, model.N], np.transpose([model.ub[0], model.ub[0]]), 'r:')
    plt.plot([0, model.N], np.transpose([model.lb[0], model.lb[0]]), 'r:')
    plt.step(range(0, model.N), u[0, :],'b-')

    # plot steering rate
    fig.add_subplot(5,2,10)
    plt.grid("both")
    plt.title('Steering rate')
    plt.plot([0, model.N], np.rad2deg(np.transpose([model.ub[1], model.ub[1]])), 'r:')
    plt.plot([0, model.N], np.rad2deg(np.transpose([model.lb[1], model.lb[1]])), 'r:')
    plt.step(range(0, model.N), np.rad2deg(u[1, :]),'b-')

    plt.tight_layout()
    plt.show()           

def main():

    # generate code for estimator
    model, solver = generate_pathplanner()

    # Simulation
    # ----------
    # Set initial guess to start solver from (here, middle of upper and 
    # lower bound)
    x0i = np.array([0.,0.,-1.5,1.5,1.,np.pi/4.,0.])
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    xinit = np.transpose(np.array([-2.,0.,0.,np.deg2rad(90),0.]))

    problem = {"x0": x0,
            "xinit": xinit}

    # Set runtime parameters
    params = np.array([-1.5,1.])
    problem["all_parameters"] = np.transpose(np.tile(params,(1,model.N)))

    # Time to solve the NLP!
    output, exitflag, info = solver.solve(problem)

    # Make sure the solver has exited properly.
    assert exitflag == 1, "bad exitflag"
    sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
        .format(info.it, info.solvetime))

    # Plot results and make interactive
    # ------------
    # extract output of solver
    temp = np.zeros((np.max(model.nvar), model.N))
    for i in range(0, model.N):
        temp[:, i] = output['x{0:02d}'.format(i+1)]
    u = temp[0:2, :]
    x = temp[2:7, :]

    # plot inputs and states and make window interactive
    plotAndMakeInteractive(x,u,model,params,problem,solver)



if __name__ == "__main__":
    main()
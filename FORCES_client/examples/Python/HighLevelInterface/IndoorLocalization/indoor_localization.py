"""
This example shows how to implement a least-squares decoder for
localization in 2D.

Assume we have noisy distance measurements d_i (noise assumed to be
zero mean and i.i.d.) from N anchors, i.e. d_i is a N-column vector.
We want to estimate the position (x,y) of a target by solving for
least-squares error

   minimize \sum_i=1^N e_i^2
 subject to  e_i == (xa-xhat)^2 + (ya-yhat)^2 - d_i^2

where (xa,ya) are the known position of the anchors (xa,ya).

(c) Embotech AG, Zurich, Switzerland, 2013-2021
"""

import numpy as np
import matplotlib.pyplot as plt
import forcespro
import forcespro.nlp

# Global model definitions
# ------------------------

n = 4  # number of anchors
xlimits = np.array([0, 10])
ylimits = np.array([0, 10])  # limits for x and y of world
xa = np.array([0, 10, 0, 10])  # x-positions for anchors
ya = np.array([0, 0, 10, 10])  # y-positions for anchors
solver = None  # will hold the solver once generated
noise = 0.1  # noise standard deviation in meters


# Generating a solver
# -------------------

def generate_estimator(number_of_anchors, xlimits, ylimits):
    """
    Generates and returns a FORCESPRO solver that esimates a position based on
    noisy measurement inputs.
    """

    # NLP problem definition
    # ----------------------

    model = forcespro.nlp.SymbolicModel(1)  # number of distance measurements
    model.nvar = 2  # number of variables (use 3 if 3D)
    model.npar = number_of_anchors * 3  # number of parameters: coordinates of anchors in 2D, plus measurements
    model.objective = objective  # objective is defined as it's own function below
    model.lb = np.array([xlimits[0], ylimits[0]])  # lower bounds on (x,y)
    model.ub = np.array([xlimits[1], ylimits[1]])  # upper bounds on (x,y)

    # FORCESPRO solver settings
    # -------------------------

    codesettings = forcespro.CodeOptions()
    codesettings.printlevel = 0  # set to 2 to see some prints
    codesettings.maxit = 50  # maximum number of iterations

    # Generate a solver
    # -----------------
    solver = model.generate_solver(codesettings)

    return solver


def objective(z, p):
    """
    This function implements the objective to be minimized.

    We assume that the parameter vector p is ordered as follows:

    - p[0:(na-1)]        - x-coordinates of the anchors
    - p[na:(2*na-1)]     - y-coordinates of the anchors
    - p[(2*na):(3*na-1)] - distance measurements of the anchors
    """
    obj = 0
    for i in range(n):
        obj += ((p[i] - z[0])**2 + (p[i + n] - z[1])**2 - p[i + 2*n]**2)**2
    return obj


def distance(xa, xtrue, ya, ytrue):
    return np.sqrt((xa - xtrue)**2 + (ya - ytrue)**2)


# Calling the solver
# ------------------

def estimate_position(event):
    """
    This callback is executed on every click inside the plot and estimates the
    position of the click from noisy measurements by calling the previously
    generated solver.
    """

    # read in true position
    xtrue = event.xdata
    ytrue = event.ydata
    assert xtrue <= xlimits[1], 'xtrue out of world'
    assert xtrue >= xlimits[0], 'xtrue out of world'
    assert ytrue <= ylimits[1], 'ytrue out of world'
    assert ytrue >= ylimits[0], 'ytrue out of world'
    plt.plot(xtrue, ytrue, 'bx', 'markersize', 8)

    # generate noisy measurements
    d = distance(xa, xtrue, ya, ytrue) + noise * np.random.rand(n)

    # feed problem data
    problem = {"x0": np.zeros((2, 1)),
               "all_parameters": np.concatenate([xa, ya, d], axis=0)}

    # solve!
    output, exitflag, info = solver.solve(problem)
    assert exitflag == 1, 'some problem in solver'  # always test exitflag for success
    xhat = output["x1"][0]
    yhat = output["x1"][1]
    esterr = np.linalg.norm(np.array([xhat, yhat]) - np.array([xtrue, ytrue]))

    # plot
    plt.plot(xhat, yhat, 'ro', 'markersize', 8)
    plt.draw()

    # print
    print('You clicked X: {0:6.4f}, Y: {1:6.4f}'.format(xtrue, ytrue))
    print('Estimated   X: {0:6.4f}, Y: {1:6.4f}'.format(xhat, yhat))
    print('This is an estimation error of {0:6.4f}. Solve time: {0:6.4f} microsceonds.'.format(esterr, info.solvetime*1E6))


def main():
    global solver  # will generate solver and store it in this global variable

    assert np.size(xa) == n, 'xa must have length {}'.format(n)
    assert all(xa >= xlimits[0]), 'xa out of world'
    assert all(ya >= ylimits[0]), 'ya out of world'
    assert all(xa <= xlimits[1]), 'xa out of world'
    assert all(ya <= ylimits[1]), 'ya out of world'

    # generate code for estimator
    solver = generate_estimator(n, xlimits, ylimits)

    # plot & make interactive
    fig = plt.gcf()
    plt.clf()
    plt.plot(xa[0], ya[0], 'kx')
    plt.plot(xa[0], ya[0], 'bx')
    plt.plot(xa[0], ya[0], 'ro')
    plt.plot(xa, ya, 'kx', 'markersize', 8)
    plt.plot(xa, ya, 'ko', 'markersize', 8)
    plt.xlim(xlimits + np.array([-1, 1]))
    plt.ylim(ylimits + np.array([-1, 1]))
    plt.axis('equal')
    plt.title('Click into figure to place target (noise level: {0:3.1f})'.format(noise))
    plt.legend(['anchors', 'true position', 'estimated position']) 
    fig.canvas.mpl_connect('button_press_event', estimate_position)
    plt.show()


if __name__ == "__main__":
    main()

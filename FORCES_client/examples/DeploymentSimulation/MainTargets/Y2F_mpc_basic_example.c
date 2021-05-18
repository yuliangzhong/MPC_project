#include <stdio.h>
#include "simpleMPC_solver/include/simpleMPC_solver.h"


int main()
{
    int return_val = 0;
    int i;
    int exit_code = 0;
    double A[2][2] = {{1.1, 1.0}, {0.0, 1.0}};
    double B[2] = {1.0, 0.5};
    double X[2] = {0.0};
    double U[1] = {0.0};
    double xinit[2] = {-4.0, 2.0};
    simpleMPC_solver_params params;
    simpleMPC_solver_info info;
    simpleMPC_solver_output output;

    for (i = 0; i < 30; i++)
    {
        X[0] = xinit[0];
        X[1] = xinit[1];
        params.xinit[0] = X[0];
        params.xinit[1] = X[1];
        exit_code = simpleMPC_solver_solve(&params, &output, &info, NULL);
        if (exit_code != 1)
        {
            printf("\n\nsimpleMPC_solver did not return optimal solution at step %d. Exiting.\n", i + 1);
            return_val = 1;
            break;
        }
        U[0] = output.u0[0];
        xinit[0] = A[0][0] * X[0] + A[0][1] * X[1] + B[0] * U[0];
        xinit[1] = A[1][0] * X[0] + A[1][1] * X[1] + B[1] * U[0];
        printf("\n\nStep %d: OPTIMAL, Iterations %d, Output U: %4.6e, State X: %4.6e %4.6e\n", i + 1, info.it, U[0], X[0], X[1]);
    }
    return return_val;
}
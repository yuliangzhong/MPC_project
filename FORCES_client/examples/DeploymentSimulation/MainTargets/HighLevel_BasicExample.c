#include <stdio.h>
#include "FORCESNLPsolver/include/FORCESNLPsolver.h"

/* CasADi - FORCESPRO interface */
extern void FORCESNLPsolver_casadi2forces(FORCESNLPsolver_float *x,        /* primal vars                                         */
                                 FORCESNLPsolver_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolver_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolver_float *p,        /* parameters                                          */
                                 FORCESNLPsolver_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolver_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolver_float *c,        /* dynamics                                            */
                                 FORCESNLPsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolver_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,      /* stage number (0 indexed)                            */
								 solver_int32_default iteration,  /* iteration number of solver                          */
								 solver_int32_default threadID    /* Id of caller thread 								 */);

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
    FORCESNLPsolver_params params;
    FORCESNLPsolver_info info;
    FORCESNLPsolver_output output;
    FORCESNLPsolver_extfunc extfunc_eval = &FORCESNLPsolver_casadi2forces;

    for (i = 0; i < 33; i++)
    {
        params.x0[i] = 0.0;
    }

    for (i = 0; i < 30; i++)
    {
        X[0] = xinit[0];
        X[1] = xinit[1];
        params.xinit[0] = X[0];
        params.xinit[1] = X[1];
        exit_code = FORCESNLPsolver_solve(&params, &output, &info, NULL, extfunc_eval);
        if (exit_code != 1)
        {
            printf("\n\nFORCESNLPsolver did not return optimal solution at step %d. Exiting.\n", i + 1);
            return_val = 1;
            break;
        }
        U[0] = output.x01[0];
        xinit[0] = A[0][0] * X[0] + A[0][1] * X[1] + B[0] * U[0];
        xinit[1] = A[1][0] * X[0] + A[1][1] * X[1] + B[1] * U[0];
        printf("\n\nStep %d: OPTIMAL, Iterations %d, Output U: %4.6e, State X: %4.6e %4.6e\n", i + 1, info.it, U[0], X[0], X[1]);
    }
    return return_val;
}
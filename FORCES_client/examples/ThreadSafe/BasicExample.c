#include "FORCES_NLP_solver/include/FORCES_NLP_solver.h"
#include <stdio.h>
#include <omp.h>
#include <stdlib.h>

/*
 * This example demonstrates how to run multiple solvers in parallel.
 * This requires the expert solve API which ensures thread-safety.
 *
 * Instructions:
 * 1) run basic_example.py from Examples/Python/HighLevelInterface with option `options.threadSafeExpert = 1`
 * 2) copy generated FORCES_NLP_solver to this directory
 * 3) compile this file with OpenMP enabled and link against solver library in FORCES_NLP_solver
 */

/* Helper function to print parameter struct */
void printParams(FORCES_NLP_solver_params *params, int iSolver)
{
    int i;
    for (i = 0; i < 2; i++)
    {
        printf("solver %i, params.xinit[%i] = %e\n", iSolver, i, params->xinit[i]);
    }
}

/* Helper function to print output struct */
void printOutput(FORCES_NLP_solver_output *output, int iSolver)
{
    int i;
    for (i = 0; i < 2; i++)
    {
        printf("solver %i, output.x01[%i] = %e\n", iSolver, i, output->x01[i]);
    }
}

int main()
{
    FORCES_NLP_solver_params params;

    params.xinit[0] = -4.;
    params.xinit[1] = 2.;

    FORCES_NLP_solver_info info;
    FORCES_NLP_solver_output output;
    solver_int32_default out_int;

/*
 * Default API (not thread-safe)
 */
    printf("\n\nDefault solve function\n--------------\n");
    printParams(&params, 0);
    out_int = FORCES_NLP_solver_solve(&params, &output, &info, NULL);
    printOutput(&output, 0);

/*
 * Expert API (thread-safe)
 * Note: 'mem' object containing all variables that are needed in the 'solve_expert' function.
 */
    printf("\n\nExpert solve function\n-------------\n");
    printParams(&params, 0);
    static FORCES_NLP_solver_mem mem;
    mem = FORCES_NLP_solver_mem_new();
    out_int = FORCES_NLP_solver_solve_expert(&params, &output, &info, &mem, NULL);
    printOutput(&output, 0);

/*
 * Expert API: 4 solvers run on multiple threads
 */
    printf("\n\nExpert solve on multiple threads\n--------------------------------\n");

/* number of solvers to be run */
#define nSolvers 4
/* maximum no of threads to run solvers concurrently */
#define nThreads 2

    omp_set_num_threads(nThreads);

    /* each thread must be assigned its own memory space */
    static FORCES_NLP_solver_mem mem_n[nThreads];

    /* input / output for each solver */
    FORCES_NLP_solver_info info_n[nSolvers];
    FORCES_NLP_solver_output output_n[nSolvers];
    FORCES_NLP_solver_params params_n[nSolvers];
    solver_int32_default exitflag[nSolvers];

#if nSolvers > 0
    params_n[0].xinit[0] = -4.;
    params_n[0].xinit[1] = 2.;
    printParams(&params_n[0], 0);
#endif
#if nSolvers > 1
    params_n[1].xinit[0] = -3.;
    params_n[1].xinit[1] = 2.;
    printParams(&params_n[1], 1);
#endif
#if nSolvers > 2
    params_n[2].xinit[0] = -1.;
    params_n[2].xinit[1] = 1.;
    printParams(&params_n[2], 2);
#endif
#if nSolvers > 3
    params_n[3].xinit[0] = -3.;
    params_n[3].xinit[1] = 1.;
    printParams(&params_n[3], 3);
#endif

    int iThread, iSolver;

    for (iThread=0; iThread<nThreads; iThread++)
    {
        mem_n[iThread] = FORCES_NLP_solver_mem_new();
    }

#pragma omp parallel for
    for (iSolver=0; iSolver<nSolvers; iSolver++)
    {
        int iThread = omp_get_thread_num();
        printf("solver %i assigned to thread %i\n", iSolver, iThread);
        exitflag[iSolver] = FORCES_NLP_solver_solve_expert(&params_n[iSolver], &output_n[iSolver], &info_n[iSolver], &mem_n[iThread], NULL);
    }

    for (iSolver=0; iSolver<nSolvers; iSolver++)
    {
        printOutput(&output_n[iSolver], iSolver);
    }

    return 0;
}

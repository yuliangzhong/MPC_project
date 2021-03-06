/*
 This is an interface for solver_name that can be used to call the solver generated by FORCES PRO
*/ 

#include "../include/solver_name.h"
#include "../include/internal_solver_name_1.h"
#include <stdio.h>

/* For Visual Studio 2015 Compatibility */
#if (_MSC_VER >= 1900)
FILE _iob[3];
FILE * __cdecl __iob_func(void)
{
    _iob[0] = *stdin;
    _iob[1] = *stdout;
    _iob[2] = *stderr;
    return _iob;
}
#endif

extern solver_int32_default solver_name_solve(solver_name_params *params, solver_name_output *output, solver_name_info *info, FILE *fs) 
{
	/* Some memory */
	static internal_solver_name_1_params params_1;
	static internal_solver_name_1_output output_1;
	static internal_solver_name_1_info info_1;

	solver_int32_default exitflag;
	/* define variables */
	solver_int32_default i;
	/* SOLVER 1 --------------------------------------------------------*/
	/*Assigning parameter values of solver #1*/
	/*Assigning parameter values of solver #1*/
	params_1.p_1[0] = 0.000000000000000000;
	params_1.p_1[1] = 0.000000000000000000;
	params_1.p_1[2] = 0.000000000000000000;
	params_1.p_1[3] = (1.000000000000000000 * params->x0[0]);
	params_1.p_1[4] = (1.000000000000000000 * params->x0[1]);
	params_1.p_1[5] = (1.000000000000000000 * params->x0[2]);

	/* call solver #1 */
	exitflag = internal_solver_name_1_solve(&params_1, &output_1, &info_1, fs );

	/* iterations */
	info->it = info_1.it;

	/* iterations to optimality (branch and bound) */
	info->it2opt = info_1.it2opt;

	/* res_eq */
	info->res_eq = info_1.res_eq;

	/* res_ineq */
	info->res_ineq = info_1.res_ineq;

	/* pobj */
	info->pobj = info_1.pobj;

	/* dobj */
	info->dobj = info_1.dobj;

	/* dgap */
	info->dgap = info_1.dgap;

	/* rdgap */
	info->rdgap = info_1.rdgap;

	/* mu */
	info->mu = info_1.mu;

	/* mu_aff */
	info->mu_aff = info_1.mu_aff;

	/* sigma */
	info->sigma = info_1.sigma;

	/* lsit_aff */

	info->lsit_aff = info_1.lsit_aff;

	/* lsit_cc */
	info->lsit_cc = info_1.lsit_cc;

	/* step_aff */
	info->step_aff = info_1.step_aff;

	/* step_cc */
	info->step_cc = info_1.step_cc;

	/* solver time */
	info->solvetime = info_1.solvetime;


	/* OUTPUTS -----------------------------------------------------------*/
	/*Build outputs*/
	output->output1[0] = (1.000000000000000000 * output_1.o_1[0]);

	output->output1[1] = (1.000000000000000000 * output_1.o_2[0]);

	output->output1[2] = (1.000000000000000000 * output_1.o_3[0]);

	return exitflag;
}
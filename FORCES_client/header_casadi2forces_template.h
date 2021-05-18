#ifdef __cplusplus
extern "C" {
#endif
    
extern void $SOLVER_NAME$_casadi2forces($SOLVER_NAME$_float *x,        /* primal vars                                         */
										 $SOLVER_NAME$_float *y,        /* eq. constraint multiplers                           */
										 $SOLVER_NAME$_float *l,        /* ineq. constraint multipliers                        */
										 $SOLVER_NAME$_float *p,        /* parameters                                          */
										 $SOLVER_NAME$_float *f,        /* objective function (scalar)                         */
										 $SOLVER_NAME$_float *nabla_f,  /* gradient of objective function                      */
										 $SOLVER_NAME$_float *c,        /* dynamics                                            */
										 $SOLVER_NAME$_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
										 $SOLVER_NAME$_float *h,        /* inequality constraints                              */
										 $SOLVER_NAME$_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
										 $SOLVER_NAME$_float *hess,     /* Hessian (column major)                              */
										 solver_int32_default stage,     /* stage number (0 indexed)                            */
										 solver_int32_default iteration, /* iteration number of solver                          */
										 solver_int32_default threadID  /* Id of caller thread */);

#ifdef __cplusplus
} /* extern "C" */
#endif
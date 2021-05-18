#include <math.h>

extern void car_dynamics(double *x, double *c);
extern void car_dynamics_jacobian(double *x, double *J);

void myfevals(  double *x,        /* primal vars */
                double *y,        /* eq. constraint multiplers */
                double *l,        /* ineq. constraint multipliers */
                double *p,        /* parameters */
                double *f,        /* objective function (incremented in this function) */
                double *nabla_f,  /* gradient of objective function */ 
                double *c,        /* dynamics */
                double *nabla_c,  /* Jacobian of the dynamics (column major) */        
                double *h,        /* inequality constraints */
                double *nabla_h,  /* Jacobian of inequality constraints (column major) */
                double *H,        /* Hessian (column major) */
                int stage,        /* stage number (0 indexed) */
                int iteration     /* iteration number of solver */)
{
    /* cost */
    if (f)
    {   /* notice the increment of f */
        (*f) += 100.*fabs(x[2]-0.0) + 100.*fabs(x[3]-3.) + 0.1*x[0]*x[0] + 0.01*x[1]*x[1];
    }
    
    /* gradient - only nonzero elements have to be filled in */
    if (nabla_f)
    {
        nabla_f[0] = 0.2*x[0];
        nabla_f[1] = 0.02*x[1];
        nabla_f[2] = 100. * fabs(x[2] - 0.) / (x[2] - 0.);
        nabla_f[3] = 100. * fabs(x[3] - 3.) / (x[3] - 3.);
    }
    
    /* eq constr */
    if (c)
    {
        car_dynamics(x, c);
    }
    
    /* jacobian equalities (column major) */
    if (nabla_c)
    {
        car_dynamics_jacobian(x, nabla_c);
    } 
    
    /* ineq constr */
    if (h)
    {
        h[0] = x[2]*x[2] + x[3]*x[3];
        h[1] = (x[2]-p[0])*(x[2]-p[0]) + (x[3]-p[1])*(x[3]-p[1]);
    }
    
    /* jacobian inequalities (column major) - only non-zero elements to be filled in */
    if (nabla_h)
    {
        /* column 3 */
        nabla_h[4] = 2.*x[2];
        nabla_h[5] = 2.*x[2] - 2*p[0];
        
        /* column 4 */
        nabla_h[6] = 2.*x[3];
        nabla_h[7] = 2.*x[3] - 2*p[1];
    } 
}
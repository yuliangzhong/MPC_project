#include <math.h>

void car_dynamics(double *x, double *c)
{   
    /* using forward euler for discretization */
    c[0] = x[2] + 0.1*x[4]*cos(x[5]+atan(0.5*tan(x[6])));
    c[1] = x[3] + 0.1*x[4]*sin(x[5]+atan(0.5*tan(x[6])));
    c[2] = x[4] + 0.1*x[0];
    c[3] = x[5] + 0.1*2.*x[4]*sin(atan(0.5*tan(x[6])));
    c[4] = x[6] + 0.1*x[1];
}

void car_dynamics_jacobian(double *x, double *J)
{   
    /* NOTE: only non-zero values in dense matrix (column major) filled in */
            
    /* 1st column: indices 0..4 */
    J[2] = 0.1;
        
    /* 2nd column: indices 5..9 */
    J[9] = 0.1;
    
    /* 3rd column: indices 10..14 */
    J[10] = 1.;
            
    /* 4th column: indices 15..19 */
    J[16] = 1.;
    
    /* 5th column: indices 20..24 */
    J[20] = 0.1*cos(x[5]+atan(0.5*tan(x[6])));
    J[21] = 0.1*sin(x[5]+atan(0.5*tan(x[6])));
    J[22] = 1.;
    J[23] = 0.1*2.*sin(atan(0.5*tan(x[6])));
    
    /* 6th column: indices 25..29 */
    J[25] = -0.1*x[4]*sin(x[5]+atan(0.5*tan(x[6])));
    J[26] =  0.1*x[4]*cos(x[5]+atan(0.5*tan(x[6])));
    J[28] = 1.;
        
    /* 7th column: indices  30..34*/
    J[30] = -2.*0.1*x[4] / (cos(x[6])*cos(x[6])) / (tan(x[6])*tan(x[6]) + 4.) * sin(atan(0.5*tan(x[6])) + x[5]);
    J[31] =  2.*0.1*x[4] / (cos(x[6])*cos(x[6])) / (tan(x[6])*tan(x[6]) + 4.) * cos(atan(0.5*tan(x[6])) + x[5]);
    J[33] =  2.*0.1*x[4] / (cos(x[6])*cos(x[6])) / (tan(x[6])*tan(x[6]) + 4.) * cos(atan(0.5*tan(x[6])));
    J[34 ] = 1.;
}

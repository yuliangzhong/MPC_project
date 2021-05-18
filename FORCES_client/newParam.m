% Shortcut code to define a parameter of the FORCESPRO solver.
% 
%    PAR = NEWPARAM(NAME, MAPS2STAGE, MAPS2DATA) returns a struct that
%    defines a valid parameter to be used with the FORCESPRO code generator,
%    where NAME is a label of the parameter, MAPS2STAGE defines to which
%    stage variable the output is mapped, and MAPS2DATA defines the
%    matrix or vector that this parameter substitutes.
%
%    PAR = NEWPARAM(NAME, MAPS2STAGE, MAPS2DATA, TYPE) returns a struct that
%    defines a valid parameter to be used with the FORCESPRO code generator, 
%	 where TYPE defines the format of the parameter. The TYPE can be set to 
%	 'diag' only for the parameter 'cost.H'. The default value of TYPE
%	 is 'dense'.
%
%    PAR = NEWPARAM(NAME, MAPS2STAGE, MAPS2DATA, MAPS2MAT) returns a struct 
%    that defines a valid parameter to be used with the FORCESPRO code generator, 
%	 where MAPS2MAT defines the number of the matrix to which this
%	 parameter belongs. This is so far only useful if you want to specify,
%	 for example, that the linear term of quadratic constraints is a
%	 parameter.
%
%    Examples: 1. To have the affine equality vector "stages(1).eq.c" as a  
%                 parameter (often used in MPC), call 
%                    
%                    par = newParam('myparameter', 1, 'eq.c');
%
%
%              2. To indicate that a parameter is used in more than one stage,
%                 you can simply put a vector of indices in the MAPS2STAGE
%                 argument of the function:
%
%                    par = newParam('Hessians', 1:N-1, 'cost.H');
%
%                 This indicates that the parameter named 'Hessians' will be
%                 the same for all stages 1 to N-1 and replaces the 'cost.H'
%                 matrix in these stages.
%
%
%			   3. To specify a diagonal Hessian call
% 
%				     par = newParam('Hessian', 1, 'cost.H', 'diag');
%
%                 The parameter size will automatically be adjusted to a
%                 column vector holding the diagonal of H.
%
%
%              4. To specify that this parameter belongs to the 3rd quadratic
%                 constraint of stage 7, and specifies its linear term, use
%
%                    par = newParam('L', 7, 'ineq.q.l', 3);
%
%
% For a detailed explanation for declaring different parameters with FORCESPRO 
% please consult the documentation at
% https://www.embotech.com/FORCES-Pro/How-to-use/MATLAB-Interface/Parametric-Problems
%
% See also MULTISTAGEPROBLEM NEWOUTPUT GENERATECODE
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

% Generate a custom solver for multistage problems using FORCESPRO.
%
%    SUCCESS = GENERATECODE(STAGES) generates, downloads and compiles your
%    custom solver for the multistage problem STAGES. Default settings are
%    used, and the default parameter is 'stages(1).eq.c', i.e. the offset
%    term in the first equality constraint (as typically used in MPC). The
%    default outputs are all stage variables.
%
%    SUCCESS = GENERATECODE(STAGES,PARAMS) does the above but with user
%    defined parameters.
%
%    SUCCESS = GENERATECODE(STAGES,PARAMS,SETTINGS) does the above but with
%    user defined parameters and settings. A settings struct can be
%    obtained by calling the function GETOPTIONS first. See the embotech
%    online documentation for a detailed list of code settings.
%
%    SUCCESS = GENERATECODE(STAGES,PARAMS,SETTINGS,OUTVARS) does the above
%    but with user defined parameters, settings and outputs. Outputs are
%    defined by an array of structs obtained by NEWOUTPUT, or you can also
%    define all variables by using GETALLOUTPUTS.
%
%    SUCCESS = GENERATECODE(STAGES,PARAMS,SETTINGS,OUTVARS,EXTRA) does the
%    above but with user defined parameters, settings, outputs and
%    additional information about the method and the interface.
%
% SEE ALSO MULTISTAGEPROBLEM NEWPARAM NEWOUTPUT GETOPTIONS
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

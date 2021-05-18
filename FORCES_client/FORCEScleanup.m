% Cleans up directory from FORCESPRO artifacts.
%
%   --- USE WITH CARE, THIS FUNCTION DELETES FILES FROM YOUR SYSTEM! ---
%   ---      WHENEVER POSSIBLE, WE TURN ON THE RECYCLE OPTION        ---
%
% 
%    FORCESCLEANUP(SOLVERNAME) removes artifacts that are placed during
%    the code generation of a solver named SOLVERNAME in the current
%    directory. It removes only files and directories that are not needed 
%    for running the solver in Matlab:
%
%       - the @CodeGen directory (from older FORCESPRO versions)
%       - ZIP file <SOLVERNAME>.zip
%       - C-files:
%           * <SOLVERNAME>_model_*.c
%           * <SOLVERNAME>_casadi2forces.c
%       - Object files:
%           * <SOLVERNAME>.[o,obj]
%           * <SOLVERNAME>_mex.[o,obj]
%           * <SOLVERNAME>_simulinkBlock.[o,obj,pdb]
%           * <SOLVERNAME>_simulinkBlockcompact.[o,obj,pdb]
%       - Temporary file for solver prints: stdout_temp 
%
%
%    FORCESCLEANUP(SOLVERNAME,'partial') is equivalent to
%    FORCESCLEANUP(SOLVERNAME)
%
%
%    FORCESCLEANUP(SOLVERNAME,'minimal') works the same way but does not 
%    delete the source files removed in the previous mode in order to be
%    able to build the solver again
%
%
%    FORCESCLEANUP(SOLVERNAME,'all') cleans up more thoroughly, deleting
%    all files placed by the code generator into the current directory:
%
%       - the @FORCESproWS directory
%       - the solverdirectory <SOLVERNAME>
%       - files created by the solver
%       - MEX FILES:
%           * <SOLVERNAME>.<mexext>
%           * <SOLVERNAME>_simulinkBlock.<mexext>
%           * <SOLVERNAME>_simulinkBlockcompact.<mexext>
%       - Matlab help file: <SOLVERNAME>.m
%       - Python interface file: <SOLVERNAME>_py.py
%
%
%    FORCESCLEANUP(SOLVERNAME, MODE, DIR) as above, but perform cleanup of 
%    the directory DIR. To perform only a partial cleanup of DIR, use 
%    MODE = 'partial'. 
%
%
%    FORCESCLEANUP(SOLVERNAME, MODE, DIR, SILENT) as above but suppresses 
%    printing if SILENT==1. Set DIR=[] (empty matrix) if you want to clean
%    the current directory.
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

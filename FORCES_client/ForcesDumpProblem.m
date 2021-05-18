% Dumps a FORCESPRO problem instance into a file to allow to exactly 
% reproduce the issues with the exported code.
%
%   ForcesDumpProblem(PROBLEM, TAG) stores the problem struct
%   into a mat or json file with standardized naming.
%
%   ForcesDumpProblem(PROBLEM, TAG, DUMPDIRECTORY, DUMPTYPE) provides 
%   additional options.
%
%       PROBLEM:        problem parameter struct
%       TAG:            optional, a unique label used inside the filename
%       DUMPDIRECTORY:  directory used to store the dumped problem instance
%       DUMPTYPE:       any ForcesDumpType specifying the information to be dumped
%
% See also FORCES_NLP, ForcesDumpFormulation, ForcesDumpType
%   
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

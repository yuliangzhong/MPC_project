% Dumps a FORCESPRO problem formulation and/or problem instance into a file 
% to allow to exactly reproduce the issues with the exported code.
%
%   ForcesDumpFormulation(FORMULATION, OPTIONS, OUTPUTS, LABEL, DUMPDIRECTORY, DUMPTYPE, PROBLEMS) 
%   stores the problem formulation and problems into a mat/json file with standardized naming. It 
%   returns a tag string for identifying the dump.
%
%       MODELORFORMULATION: For ForcesDumpType.LegacyDumpGeneratedC: 
%                           formulation struct as returned as third argument by FORCES_NLP                    
%                           For ForcesDumpType.DumpSymbolics:
%                           model as provided to FORCES_NLP
%       OPTIONS:            codeoptions as provided to FORCES_NLP
%       OUTPUTS:            outputs as provided to FORCES_NLP
%       LABEL:              optional, a custom label used inside the filename
%       DUMPDIRECTORY:      directory used to store the dumped problem formulation
%       PROBLEMS:           a cell array of problem structs
%       DUMPTYPE:           any ForcesDumpType specifying the information to be dumped
%       VARARGIN:           any additional data to be stored in the dump
%
% See also FORCES_NLP, ForcesDumpFormulation, ForcesDumpProblem, ForcesDumpType
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

% Dumps a FORCESPRO problem formulation into a file to allow to exactly 
% reproduce the issues with the exported code.
%
%   ForcesDumpFormulation(FORMULATION, OPTIONS, OUTPUTS) stores the 
%   problem formulation into a mat or json file with standardized naming. 
%   It returns  a tag string that should be passed to ForcesDumpProblem  
%   when dumping actual problem instances.
%
%   ForcesDumpFormulation(FORMULATION, OPTIONS, OUTPUTS, LABEL, DUMPDIRECTORY, DUMPTYPE)
%   provides addtional options.
%
%       MODELORFORMULATION:  For ForcesDumpType.LegacyDumpGeneratedC: 
%                            formulation struct as returned as third argument by FORCES_NLP                    
%                            For ForcesDumpType.DumpSymbolics:
%                            model as provided to FORCES_NLP or formulation
%                            struct
%       OPTIONS:             codeoptions as provided to FORCES_NLP
%       OUTPUTS:             outputs as provided to FORCES_NLP
%       LABEL:               optional, a custom label used inside the filename
%       DUMPDIRECTORY:       directory used to store the dumped problem formulation
%       DUMPTYPE:            any ForcesDumpType specifying the information to be dumped
%
% See also FORCES_NLP, ForcesDumpProblem, ForcesDumpType
% 
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

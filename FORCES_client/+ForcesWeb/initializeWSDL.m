% Initialize WSDL Class to contact the FORCESPRO code generation server
%
% OBJ = initializeWSDL() initializes the WSDL class for the FORCESPRO
% codegen web service and returns wsdl OBJ (no arguments only work with legacy communication)
%
% OBJ = initializeWSDL(SERVER) will use the SERVER url to communicate
% with the service for the new version of network communication
% (first argument necessary for new version of communication)
%
% OBJ = initializeWSDL(SERVER, LEGACYVERSION) will use the 
% old version for network communication if LEGACYVERSION is true
%
% See also ForcesWeb generateWSDL finalizeWSDL
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

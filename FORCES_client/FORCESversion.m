% Returns the version string of the FORCESPRO client or a server.
%
%    VER = FORCESVERSION() without any arguments returns the version of the 
%          FORCESPRO client as a string.
%
%    VER = FORCESVERSION(SERVER) retrieves the server version as a
%          string. An error is thrown if either no connection can be made, 
%          or the corresponding server does not publish its version.
%
%    [VER, OFFLINEDATE] = FORCESVERSION() also returns the date when the 
%          codegen server corresponding to the current client version is
%          planned to go offline.
%
% See also UPDATECLIENT
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

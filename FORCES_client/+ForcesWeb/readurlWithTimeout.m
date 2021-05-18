% Read content from the given url
%
% [DATA, TIMEDOUT] = readurlWithTimeout(URL, TIMEOUT) reads the content 
% from the given URL and returns it to the character array DATA. 
% If the operation did not finish before the default timeout 
% the operation returns true in TIMEDOUT
%
% [DATA, TIMEDOUT] = readurlWithTimeout(URL, TIMEOUT) reads the content 
% from the given URL and returns it to the character array DATA. 
% If the operation did not finish before the set TIMEOUT (in sec) 
% the operation returns true in TIMEDOUT
%
% [DATA, TIMEDOUT] = readurlWithTimeout(_, LEGACYVERSION) will use the 
% old version for network communication if LEGACYVERSION is true.
%
% See also ForcesWeb fileurl download fileNotFoundException
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

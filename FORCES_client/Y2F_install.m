function Y2F_install(version, silent, target)
% Download Y2F, the Yalmip-FORCES interface, and add it to the Matlab path.
%
%    Y2F_INSTALL() downloads the master branch of Y2F and unpacks 
%    it into the directory 'Y2F'. If that directory exists, the user is 
%    prompted to confirm deleting the existing directory. The directory is
%    added to the Matlab path, such that Y2F can be used from any
%    directory.
%
%    Y2F_INSTALL(VERSION) as above, downloads the tag 'v{VERSION}' of Y2F.
%    If VERSION is an empty matrix [], the master branch will be retrieved.
%
%    Y2F_INSTALL(VERSION, SILENT) As above, and for SILENT==TRUE no prompt 
%    will be displayed - any existing directory will be deleted.
%
%    Y2F_INSTALL(VERSION, SILENT, TARGET) As above, but the contents of Y2F
%    will be placed into the directory defined in the string TARGET.
%
% See also Y2F_DOWNLOAD
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

%% download y2f
switch( nargin )
    
    case 0
        y2f_directory = Y2F_download;
        
    case 1        
        y2f_directory = Y2F_download(version);
    
    case 2        
        y2f_directory = Y2F_download(version, silent);
    
    otherwise 
        y2f_directory = Y2F_download(version, silent, target);
        
end

%% add to path
addpath(genpath(y2f_directory));
savepath;

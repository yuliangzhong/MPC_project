function blkData = forcesmpcblock_MaskInitFcn(blk)
%% FORCESPRO "mpc-toolbox-plugin" Simulink block utility

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.

% Blocks use two functions in the following order to initialize:
%
%   InitFcn: validate @mpc object, cross-check signal consistency and
%   connection. Create core data from object and save a structure to block
%   "UserData" proerty, which will be used by "MaskInitFcn". 
%
%   MaskInitFcn: when "UserData" is empty (i.e. model opens or no
%   coredata), initialize the block with dummy dimensions.  Otherwise,
%   create mask variables from "UserData".  Modify the block I/O based on
%   dialog settings.  Each constant block (when an optional inport is off)
%   is initialized with correct dimension.

function [maskdisplay, port] = forcesmpcblock_UpdateInport(blk,AvailableBlocks,type_in_mask,type_in_block,index,name,port,mpcport,maskdisplay)
% FORCESPRO MPC block uses three functions to initialize at compile time (in
% the following order):
%
%   InitFcn: collect data from "coredata" and "statedata" structures
%   specified in the block dialog.  Check inport connectivity.  Create
%   "MPCstruct" and save it to block "UserData".
%
%   MaskInitFcn: split into two functions: Parameter and Resize.
%
%       Parameter: when "UserData" is empty (i.e. model opens or no @mpc),
%       no initialization is needed.  Otherwise, create mask variables from
%       "UserData" with additional sanity check and memory optimization.
%
%       Resize: modify the block I/O based on dialog settings.  Each
%       constant block (when an optional inport is off) is initialized with
%       correct dimension.

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.

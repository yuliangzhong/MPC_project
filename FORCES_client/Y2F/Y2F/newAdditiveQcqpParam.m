function param = newAdditiveQcqpParam( maps2index, maps2origparam, maps2mat, factor )
%NEWQCQPPARAM Creates new additive parameter for QCQPs
% structure:
%     param.maps2index      index of QCQP matrix elements that is affected
%                           by parameter
%     param.maps2origparam  index of original parameter (used to recover
%                           value) 
%     param.factor          factor by which parameter value has to be
%                           multiplied before it is added
%     param.maps2mat        index of QCQP matrix (only relevant for quad.
%                           constraints) 
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

param.maps2index = maps2index;

if nargin >= 2
    param.maps2origparam = maps2origparam;
else
    param.maps2origparam = 0;   
end

if nargin >= 3
    param.maps2mat = maps2mat;
else
    param.maps2mat = 1;
end

if nargin >= 4
    param.factor = factor;
else
    param.factor = 1;
end


end


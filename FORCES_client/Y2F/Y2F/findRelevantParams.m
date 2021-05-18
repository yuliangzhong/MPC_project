function idx = findRelevantParams( rowIdx, colIdx, matrixSize, qcqpParamList, maps2mat )
%FINDRELEVANTPARAMS Helper function that finds QCQP parameters that apply
%to given (subscript) indices (and matrix) inside given parameter list.
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

% convert subscripts to linear indices
linIndices = [];
if all(matrixSize > 1)
    for c=colIdx
        for r=rowIdx
            linIndices(end+1) = sub2ind(matrixSize,r,c);
        end
    end
elseif matrixSize(2) == 1
    linIndices = rowIdx;
elseif matrixSize(1) == 1
    linIndices = colIdx;
end

% compare linear indices to index of every QCQP param
idx = [];
for i=1:numel(qcqpParamList)
    if any(ismember(qcqpParamList(i).maps2index, linIndices)) && ...
            (nargin==4 || qcqpParamList(i).maps2mat == maps2mat)
        idx(end+1) = i;
    end
end

end


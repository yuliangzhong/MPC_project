function [vertexIdx,localIdx,componentIdx] = findVariableIndex( components, variable )
%FINDVARIABLEINDEX Returns index of vertex where the given variable is
% located, the position inside the vertex, and the corresponding component
% index. If variable cannot be found this function returns 0.
% 
% Input:
%   components  cell array of connected graphs
%   variable    variable number
%   
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

if ~iscell(components)
    components = {components};
end

for k=1:numel(components)
    G = components{k};
    vertexIdx = 0;
    componentIdx = k;
    for i=1:numel(G.vertices)
        localIdx = find(G.vertices{i} == variable,1);
        if ~isempty(localIdx)
            vertexIdx = i;
            return
        end
    end
end

end


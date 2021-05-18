function result = checkIfGraphIsPathGraph( G )
%CHECKGRAPHISPATHGRAPH Checks if a graph is a path graph
%   result = 1 if given graph is a path graph, 0 otherwise
%   G has the format described in EMPTYGRAPH
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

result = 0;

if ~checkIfGraphIsConnected(G)
    return
end

% Simple cases
if G.n <= 1
    result = 1;
    return
end

% Do we have the right degrees? We need two 1s and the rest 2s
deg = sum(G.adjMatrix);
% Find the 1s
ones = find(deg == 1);
if length(ones) == 2
    deg(ones) = [];
    if all(deg == 2)
        result = 1;
    end
end

end


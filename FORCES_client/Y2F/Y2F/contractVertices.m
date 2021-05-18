function G = contractVertices( G, vertex1, vertex2 )
%CONTRACTVERTICES Contract two vertices of a graph
%   graph = graph data structure (see EMPTYGRAPH for more information)
%   vertex1 = index of first vertex
%   vertex2 = index of second vertex
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

if vertex1 == vertex2 
    return % nothing to do
end

% We want to keep vertices in order --> switch if necessary
if G.vertices{vertex1}(1) > G.vertices{vertex2}(1)
    v = vertex1;
    vertex1 = vertex2;
    vertex2 = v;
end

% Fix edges
G.adjMatrix(vertex1,:) = G.adjMatrix(vertex1,:) | G.adjMatrix(vertex2,:);
G.adjMatrix(:,vertex1) = G.adjMatrix(:,vertex1) | G.adjMatrix(:,vertex2);
G.adjMatrix(vertex1,vertex1) = 0;
G.adjMatrix(:,vertex2) = [];
G.adjMatrix(vertex2,:) = [];

% Merge variable sets
G.vertices{vertex1} = sort([G.vertices{vertex1} G.vertices{vertex2}]);
G.vertices(vertex2) = [];

G.n = G.n - 1;

end


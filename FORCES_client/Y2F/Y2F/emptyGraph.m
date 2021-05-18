function G = emptyGraph()
%EMPTYGRAPH returns an empty graph G
% Graphs are stored as structs with the following fields:
%   .vertices   cell array of vertex labels (lists of variable indices)
%   .adjMatrix  adjacency matrix
%   .n          number of vertices
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

G.vertices = {};
G.adjMatrix = [];
G.n = 0;

end


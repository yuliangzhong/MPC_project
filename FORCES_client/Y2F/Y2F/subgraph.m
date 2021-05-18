function G = subgraph( G, idx )
%SUBGRAPH Returns the induced subgraph of G containing the vertices with
% indices in idx
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

G.vertices = G.vertices(idx);
G.adjMatrix = G.adjMatrix(idx, idx);
G.n = length(idx);

end


function components = splitGraphIntoComponents( G )
%SPLITGRAPHINTOCOMPONENTS splits graph G up into connected subgraphs using
%a breadth-first search. Returns a cell array containing all subgraphs. See
%EMPTYGRAPH for graph format.
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

if G.n <= 1
    components = {G};
    return
end

components = {};
c = 1;

verticesNotVisited = 1:G.n;

while ~isempty(verticesNotVisited)
    verticesForGraph = [];
    nextVertices = verticesNotVisited(1);
    verticesNotVisited = verticesNotVisited(2:end);
    while ~isempty(nextVertices)
        v = nextVertices(1);
        idx = intersect(find(G.adjMatrix(v,:)), verticesNotVisited);
        verticesNotVisited = setdiff(verticesNotVisited, idx);
        nextVertices = [nextVertices(2:end) idx];
        verticesForGraph = [verticesForGraph v];
    end
    verticesForGraph = sort(unique(verticesForGraph));
    components{c} = subgraph(G,verticesForGraph);
    c = c+1;
end

end


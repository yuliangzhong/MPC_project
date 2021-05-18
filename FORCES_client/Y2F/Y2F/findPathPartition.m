function partition = findPathPartition( G )
%FINDPATHPARTITION Applies path partitioning algorithm to find a maximal
%path partition of the graph G. G has the format described in EMPTYGRAPH.
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

if G.n == 1
    partition = G;
    return
end

% order vertices according to their degree --> lower degree vertices are
% preferred as starting vertices (likely to belong to first or last stage)
indicies = 1:G.n;
deg = sum(G.adjMatrix);
[~,I] = sort(deg);
indicies = indicies(I);

% Start path partitioning with every vertices and compare number of subsets
best_vertices = {};
min_cost = 0;
for i=indicies
    % Path partitioning ("Algorithm 1" in the thesis)
    v = i;
    vertices = {[G.vertices{v}]};
    remaining_v = ones(1,G.n);
    remaining_v(v) = 0;
    while any(remaining_v)
        next_v = find(any(G.adjMatrix(v,:),1) & remaining_v);
        vertices{end+1} = [G.vertices{next_v}];
        remaining_v(next_v) = 0;
        v = next_v;
    end
    
    % Compare number of subsets with current best
    cost = costOfGraphVertices(vertices);
    if isempty(best_vertices) || numel(vertices) > numel(best_vertices) || cost < min_cost
        best_vertices = vertices;
        min_cost = cost;
    end
end

% Create new graph from path partitioning
partition = emptyGraph;
partition.vertices = best_vertices;
partition.n = numel(best_vertices);
partition.adjMatrix = zeros(partition.n);
partition.adjMatrix(partition.n+1:partition.n+1:end) = 1;
partition.adjMatrix(2:partition.n+1:end) = 1;

end

function cost = costOfGraphVertices(vertices)
% Computes a cost that can be used to compare different path graphs
% Cost depends on number of subsets and variables in those subsets

cost = sum(cellfun(@length,vertices).^3);

end


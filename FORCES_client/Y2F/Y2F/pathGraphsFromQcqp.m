function graphs = pathGraphsFromQcqp(H,Aineq,Aeq,Q,l)
%PATHGRAPHSFROMQCQP Creates path graphs from a given QCQP using path
% partitioning algorithm.
% Input:
%   H,Aineq,Aeq,Q,l are standard QCQP matrices
% Output:
%   cell array of graphs (see EMPTYGRAPH for graph format)
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

% Construct graph from equality constraints
G = emptyGraph;
n = size(H,1);
G.vertices = num2cell(1:n);
G.adjMatrix = generateAdjMatrixFromEqualities(Aeq);
G.n = n;

% Variables have to be in the same stage if they appear in the same cost
% term or linear/quadratic inequality
G = simplifyGraphWithCost(G,H);
G = simplifyGraphWithIneq(G,Aineq);
for k=1:numel(Q)
    G = simplifyGraphWithQuadIneq(G,Q{k},l(:,k));
end

if ~checkIfGraphIsConnected(G)
    beep
    warning('Y2F:separableProblem','Problem is separable. Consider using multiple solvers.');
end

% Find components of graph
graphs = splitGraphIntoComponents(G);

% Use partitioning algorithm to convert each component into path graph
for i=1:numel(graphs)
    graphs{i} = findPathPartition(graphs{i});
end

end
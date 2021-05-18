function G = simplifyGraphWithQuadIneq( G, Q, l )
%SIMPLIFYGRAPHWITHQUADINEQ contracts vertices that contain variables in
%their labels that appear in the same quadratic inequality.
%Example: x2*x3 + ... <= 5 --> vertices with x2 and x3 in their
%respective label are contracted
%Input:
%   G       graph (format see EMPTYGRAPH)
%   Q, l   	matrices of LHS of a quadratic inequality
%Output:
%   modified graph G
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

if  ~isempty(Q) && size(Q,1)~=size(Q,2)
    error('Q has to be quadratic')
end

if ~isvector(l)
    error('l has to be a vector')
end

idx = [];
for r=1:size(Q,1)
    idx = [idx find(Q(r,:)~=0)];
end
idx = [idx find(l'~=0)];

idx = sort(unique(idx));

i1 = findVariableIndex(G,idx(1));
for v2 = idx(end:-1:2)
    i2 = findVariableIndex(G,v2);
    G = contractVertices(G,i1,i2);
end

end


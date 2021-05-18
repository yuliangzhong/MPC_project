function G = simplifyGraphWithCost( G, H )
%SIMPLIFYGRAPHWITHCOST contracts vertices that contain variables in
%their labels that appear in the same cost term.
%Example: cost = ... + x2 * x3 + ... --> vertices with x2 and x3 in their
%respective label are contracted
%Input:
%   G   graph (format see EMPTYGRAPH)
%   H   cost matrix (Hessian)
%Output:
%   modified graph G
% 
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

if ~isempty(H) && size(H,1)~=size(H,2)
    error('H has to be quadratic')
end

for r=1:size(H,1)
    for c=find(H(r,:)~=0)
        if c~=r
            i1 = findVariableIndex(G,r);
            i2 = findVariableIndex(G,c);
            G = contractVertices(G,i1,i2);
        end
    end
end

end


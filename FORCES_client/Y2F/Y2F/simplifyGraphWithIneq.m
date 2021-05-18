function G = simplifyGraphWithIneq( G, Aineq )
%SIMPLIFYGRAPHWITHINEQ contracts vertices that contain variables in
%their labels that appear in the same linear inequality.
%Example: x2 + x3 + ... <= 5 --> vertices with x2 and x3 in their
%respective label are contracted
%Input:
%   G       graph (format see EMPTYGRAPH)
%   Aineq   linear inequality matrix
%Output:
%   modified graph G
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

for i=1:size(Aineq,1)
    idx = find(Aineq(i,:)~=0);
    if ~isempty(idx)
        v1 = idx(1);
        for v2=idx(2:end)
            i1 = findVariableIndex(G,v1);
            i2 = findVariableIndex(G,v2);
            G = contractVertices(G,i1,i2);
        end
    end
end

end


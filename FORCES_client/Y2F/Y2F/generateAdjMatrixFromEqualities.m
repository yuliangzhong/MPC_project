function adj = generateAdjMatrixFromEqualities( Aeq )
%GENERATEADJMATRIXFROMEQUALITIES Generates adjacency matrix from equality
%constraints. If two variables appear in the same equality, the
%corresponding vertices are connected.
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

n = size(Aeq,2);
adj = double(Aeq ~= 0); % temp result
adj = (adj'*adj) > 0;
adj(1:n+1:end) = 0; % diagonal has to be 0

end


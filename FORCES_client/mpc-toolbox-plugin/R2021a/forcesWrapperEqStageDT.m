function eq = dummy(z,p,stage,dimension,statefcn,eqfcn)
% get inputs
[x,u] = forcesgetxufromz(z,p,stage,dimension);
% get statefcn parameter
if dimension.npstate>0
    state_para = p(dimension.paraIndex);
else
    state_para = [];
end
% equality constraints include:
%   Inter-stage state transition: x[k+1] = f(x[k], u[k])
%   Custom EqConFcn
if isempty(eqfcn)
    if dimension.npstate>0
        eq = statefcn(x,u,state_para);
    else
        eq = statefcn(x,u);
    end
else
    if dimension.npstate>0
        eq1 = statefcn(x,u,state_para);
    else
        eq1 = statefcn(x,u);
    end
    if dimension.nps(stage)>0
        stage_para = p(dimension.pIndex{stage});
        eq2 = eqfcn(stage,x,u,stage_para);
    else
        eq2 = eqfcn(stage,x,u);
    end
    eq = [eq1;eq2];
end


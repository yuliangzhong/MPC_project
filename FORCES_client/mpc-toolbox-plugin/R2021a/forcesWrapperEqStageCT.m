function eq = dummy(z,p,stage,dimension,statefcn,eqfcn,Ts,M)
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
    eq = IRK2(x,u,(1:dimension.nx),(1:dimension.nu)+dimension.nx,statefcn,Ts,state_para,M);
else
    eq1 = IRK2(x,u,(1:dimension.nx),(1:dimension.nu)+dimension.nx,statefcn,Ts,state_para,M);
    if dimension.nps(stage)>0
        stage_para = p(dimension.pIndex{stage});
        eq2 = eqfcn(stage,x,u,stage_para);
    else
        eq2 = eqfcn(stage,x,u);
    end
    eq = [eq1;eq2];
end


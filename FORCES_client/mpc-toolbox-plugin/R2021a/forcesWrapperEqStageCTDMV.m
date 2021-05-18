function eq = dummy(z,p,stage,dimension,statefcn,eqfcn,Ts,M)
% get inputs
[x,u,dmv,mv1] = forcesgetxudmvfromz(z,p,stage,dimension);
% get statefcn parameter
if dimension.npstate>0
    state_para = p(dimension.paraIndex);
else
    state_para = [];
end
% equality constraints include:
%   Inter-stage state transition: x[k+1] = f(x[k], u[k])
%   DMV definition: mv[k] = mv[k-1] + dmv[k]
%   Custom EqConFcn
if isempty(eqfcn)
    eq1 = IRK2(x,u,(1:dimension.nx),(1:dimension.nu)+dimension.nx,statefcn,Ts,state_para,M);
    eq2 = mv1 + dmv;
    eq = [eq1; eq2];
else
    eq1 = IRK2(x,u,(1:dimension.nx),(1:dimension.nu)+dimension.nx,statefcn,Ts,state_para,M);
    eq2 = mv1 + dmv;
    if dimension.nps(stage)>0
        stage_para = p(dimension.pIndex{stage});
        eq3 = eqfcn(stage,x,u,dmv,stage_para);
    else
        eq3 = eqfcn(stage,x,u,dmv);
    end
    eq = [eq1;eq2;eq3];
end


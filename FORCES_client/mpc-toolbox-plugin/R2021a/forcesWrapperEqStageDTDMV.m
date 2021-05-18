function eq = dummy(z,p,stage,dimension,statefcn,eqfcn)
% Inter-stage state transition: x[k+1] = f(x[k], u[k])
[x,u,dmv,mv1] = forcesgetxudmvfromz(z,p,stage,dimension);
% get para
if dimension.npstate>0
    state_para = p(dimension.paraIndex);
else
    state_para = [];
end
% Additional equality constraints
if isempty(eqfcn)
    if dimension.npstate>0
        eq1 = statefcn(x,u,dmv,state_para);
    else
        eq1 = statefcn(x,u,dmv);
    end
    eq2 = mv1 + dmv;
    eq = [eq1; eq2];
else
    if dimension.npstate>0
        eq1 = statefcn(x,u,dmv,state_para);
    else
        eq1 = statefcn(x,u,dmv);
    end
    eq2 = mv1 + dmv;
    if dimension.nps(stage)>0
        stage_para = p(dimension.pIndex{stage});
        eq3 = eqfcn(stage,x,u,dmv,stage_para);
    else
        eq3 = eqfcn(stage,x,u,dmv);
    end
    eq = [eq1;eq2;eq3];
end


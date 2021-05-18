function ineq = dummy(z,p,stage,dimension,ineqfcn)
% get inputs
[x,u] = forcesgetxufromz(z,p,stage,dimension);
% inequality constraints come from IneqConFcn
if dimension.nes(stage)>0
    e = z(dimension.eIndex{stage});
    if dimension.nps(stage)>0
        para = p(dimension.pIndex{stage});
        ineq = ineqfcn(stage,x,u,e,para);
    else
        ineq = ineqfcn(stage,x,u,e);
    end
else
    if dimension.nps(stage)>0
        para = p(dimension.pIndex{stage});
        ineq = ineqfcn(stage,x,u,para);
    else
        ineq = ineqfcn(stage,x,u);
    end
end


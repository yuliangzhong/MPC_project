function ineq = dummy(z,p,stage,dimension,ineqfcn)
% get inputs
[x,u,dmv] = forcesgetxudmvfromz(z,p,stage,dimension);
% inequality constraints come from IneqConFcn
if dimension.nes(stage)>0
    e = z(dimension.eIndex{stage});
    if dimension.nps(stage)>0
        para = p(dimension.pIndex{stage});
        ineq = ineqfcn(stage,x,u,dmv,e,para);
    else
        ineq = ineqfcn(stage,x,u,dmv,e);
    end
else
    if dimension.nps(stage)>0
        para = p(dimension.pIndex{stage});
        ineq = ineqfcn(stage,x,u,dmv,para);
    else
        ineq = ineqfcn(stage,x,u,dmv);
    end
end


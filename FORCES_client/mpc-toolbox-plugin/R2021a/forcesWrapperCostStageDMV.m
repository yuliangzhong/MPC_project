function cost = dummy(z,p,stage,dimension,costfcn)
% z = [x mv dmv e] for stage 1..N-1 and [x mv e] for stage N
% p = [stage md state_para stage_para]
% calculate cost based on presence of slack variables and stage parameters
[x,u,dmv] = forcesgetxudmvfromz(z,p,stage,dimension);
% evaluate
if dimension.nes(stage)>0
    e = z(dimension.eIndex{stage});
    if dimension.nps(stage)>0
        para = p(dimension.pIndex{stage});
        cost = costfcn(stage,x,u,dmv,e,para);
    else
        cost = costfcn(stage,x,u,dmv,e);
    end
else
    if dimension.nps(stage)>0
        para = p(dimension.pIndex{stage});
        cost = costfcn(stage,x,u,dmv,para);
    else
        cost = costfcn(stage,x,u,dmv);
    end
end

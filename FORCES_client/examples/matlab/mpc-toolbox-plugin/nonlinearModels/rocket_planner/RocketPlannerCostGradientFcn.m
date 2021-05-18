function [Gx, Gmv, Gdmv] = RocketPlannerCostGradientFcn(stage,x,u,dmv)
% Rocket planner cost gradient function.

% Copyright 2020-2021 The MathWorks, Inc.

Gx = zeros(6,1);
Gmv = ones(2,1);
Gdmv = zeros(2,1);
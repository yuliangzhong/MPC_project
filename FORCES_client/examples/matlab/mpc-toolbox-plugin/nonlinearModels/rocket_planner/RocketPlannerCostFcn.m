function J = RocketPlannerCostFcn(stage,x,u)
% Rocket planner cost function.

% Copyright 2020-2021 The MathWorks, Inc.

%% The control of the rocket lander consists of the force of the left thrust on 
% the left and the force of the thrust on the right (both in Newton). The 
% cost fcn is simply the sum of these forces.

J = 0;
for ct = 1:length(u)
    J = J + u(ct);
end

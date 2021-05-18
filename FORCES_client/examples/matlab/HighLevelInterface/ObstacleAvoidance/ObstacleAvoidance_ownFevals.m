% Example script for getting started with FORCESPRO NLP solver.
% 
% --------------------------------------------------------------------------
% NOTE: This example shows how to pass C functions implementing the
% function evaluations to FORCES. There is an automated way of creating
% these C functions, which is explained in the other example file named
% "ObstacleAvoidance.m"
% --------------------------------------------------------------------------
% 
% This example solves an optimization problem for a car with the simple
% continuous-time, nonlinear dynamics (bicycle model):
% 
%    dxPos/dt  = v*cos(theta + beta)
%    dyPos/dt  = v*sin(theta + beta)
%    dv/dt     = F/m
%    dtheta/dt = v/l_r*sin(beta)
%    ddelta/dt = phi
% 
%    with:
%    beta      = arctan(l_r/(l_f + l_r)*tan(delta))
% 
% where xPos,yPos are the position, v the velocity in heading angle theta 
% of the car, and delta is the steering angle relative to the heading 
% angle. The inputs are acceleration force F and steering rate phi. The 
% physical constants m, l_r and l_f denote the car's mass and the distance 
% from the car's center of gravity to the rear wheels and the front wheels.
% 
% The car starts from standstill with a certain heading angle, and the
% optimization problem is to move the car as close as possible to a certain
% end position while staying inside a nonconvex area. In addition, an 
% elliptic obstacle must be avoided. Using an interactive window, 
% the position of the obstacle can be changed by the user and the car's 
% trajectory is adapted automatically.
% 
% Quadratic costs for the acceleration force and steering are added to the
% objective to avoid excessive maneouvers.
% 
% There are bounds on all variables except theta.
% 
% Variables are collected stage-wise into 
% 
%     z = [F phi xPos yPos v theta delta].
% 
% See also FORCES_NLP
% 
% (c) Embotech AG, Zurich, Switzerland, 2013-2021.


function pathCalculator

    close all; clc;

    %% Generate code for path finder
    model = generatePathplanner();

    %% Calculate path for fixed obstacle
    % Set runtime parameters
    params = [-1.5; 1.0];
    problem.all_parameters = repmat(params,model.N,1);

    % Set initial guess to start solver from (here, middle of upper and lower
    % bound)
    x0i=[0.0,0.0,-1.5,1.5,1.,pi/4.,0.];
    x0=repmat(x0i',model.N,1);
    problem.x0=x0; 

    % Set initial conditions. This is usually changing from problem instance 
    % to problem instance:
    model.xinit = [-2., 0., 0., deg2rad(90), 0.]';
    problem.xinit = model.xinit; % xPos=-2, yPos=0, v=0 (standstill), 
    % heading angle=90°, steering angle=0°

    % Time to solve the NLP!
    [output,exitflag,info] = FORCESNLPsolver(problem);

    % Make sure the solver has exited properly. 
    assert(exitflag == 1,'Some problem in FORCESPRO solver');

    % extract output
    u = zeros(2,model.N);
    x = zeros(5,model.N);
    for i=1:model.N
        temp = output.(['x',sprintf('%02d',i)]);
        x(:,i) = temp(3:7,1);
        u(:,i) = temp(1:2,1);
    end

    %% Plot results and make window interactive
    plotAndMakeInteractive(x,u,model,params);
end

function [model] = generatePathplanner()
% Formulates the optimization problem, generates a solver for the path 
% planning problem by calling the FORCESPRO code generation.
    
    %% Problem dimensions
    model.N = 50;            % horizon length
    model.nvar = 7;          % number of variables
    model.neq  = 5;          % number of equality constraints
    model.nh = 2;            % number of inequality constraint functions
    model.npar = 2;          % number of runtime parameters

    %% Define source file containing function evaluation code
    model.extfuncs = 'C/myfevals.c';
    % Note: we will add additional source files required below at code 
    % options!

    % Indices on LHS of dynamical constraint - for efficiency reasons, make
    % sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = [zeros(5,2), eye(5)];

    %% Inequality constraints
    % upper/lower variable bounds lb <= z <= ub
    %            inputs             |             states
    %             F      phi           xPos  yPos     v    theta    delta
    model.lb = [ -5.0,  deg2rad(-40),  -3.,   0.,   0.,  -inf,   -0.48*pi]; 
    model.ub = [ +5.0,  deg2rad(+40),   0.,   3.,   2.,  +inf,    0.48*pi];

    % General (differentiable) nonlinear inequalities hl <= h(z,p) <= hu
    % model.ineq = @(z,p)  [   z(3)^2 + z(4)^2; 
    %                        (z(3)-p(1))^2 + (z(4)-p(2))^2 ];

    % Upper/lower bounds for inequalities
    model.hu = [9, +inf]';
    model.hl = [1, 0.7^2]';

    %% Initial conditions
    % Initial conditions on all states
    model.xinitidx = 3:7; % use this to specify on which variables initial
    % conditions are imposed
   
    %% Define solver options
    codeoptions = getOptions('FORCESNLPsolver');
    codeoptions.maxit = 400;    % Maximum number of iterations
    codeoptions.printlevel = 2; % Use printlevel = 2 to print progress (but 
    % not for timings)
    codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size,
    % 2: optimize for speed, 3: optimize for size & speed
    codeoptions.cleanup = false;
    codeoptions.timing = 1;
    codeoptions.printlevel = 0;
    codeoptions.nlp.bfgs_init = 3.000 * eye(7); % set initialization of the 
    % hessian approximation
    % change this to your server or leave uncommented for using the 
    % standard embotech server at https://www.embotech.com/codegen
    % codeoptions.server = 'https://yourforcesserver.com:1234';
    codeoptions.nlp.other_srcs = 'C/car_dynamics.c';
    
    %% Generate FORCESPRO solver
    FORCES_NLP(model, codeoptions);

end

function rad = deg2rad(deg) % convert degrees into radians
    rad = deg/180*pi;
end

function deg = rad2deg(rad) % convert radians into degrees
    deg = rad/pi*180;
end


%% Callback for plot
function calculatePath(hObject,~,model)
% Run solver with the new clicked obstacle position and
% update the plots
    
    % read in parameters
    pos=get(gca,'CurrentPoint');
    param1=pos(1,1);
    param2=pos(1,2);
    fprintf([newline 'You clicked X: %4.4f, Y: %4.4f'],param1,param2);
    
     if ((param1 - model.xinit(1))^2 + (param2 - model.xinit(2))^2 ...
             <= model.hl(2))
        fprintf([newline ...
            'The initial position cannot lie within obstacle. ' newline]);
     elseif (param1 < model.lb(3) || param1 > model.ub(3) || ...
        param2 < model.lb(4) || param2 > model.ub(4))
        fprintf([newline 'The obstacle position you clicked is out of '...
            'bounds' newline]);
     else
        % Set runtime parameters
        params = [param1; param2];
        problem.all_parameters = repmat(params,model.N,1);

        % Set initial guess to start solver from:
        x0i=[0.0,0.0,-1.5,1.5,1.,pi/4.,0.];
        x0=repmat(x0i',model.N,1);
        problem.x0=x0; 

        % Set initial conditions. This is usually changing from problem 
        % instance to problem instance
        problem.xinit = model.xinit;
      
        % Time to solve the NLP!
        [output,exitflag,info] = FORCESNLPsolver(problem);

        % Make sure the solver has exited properly. 
         if (exitflag < 0)
            fprintf([newline 'Some error in FORCESPRO solver. exitflag=%d'...
                newline],exitflag)
         elseif (exitflag == 0)
            fprintf([newline 'Error in FORCESPRO solver: Maximum number'...
                ' of iterations was reached. ' newline]);
         else
            fprintf([newline 'FORCESPRO took %d iterations and %f seconds '...
                'to solve the problem.' newline], info.it, info.solvetime);
            % extract the output
            temp = zeros(model.nvar,model.N);
            for i=1:model.N
                temp(:,i) = output.(['x',sprintf('%02d',i)]);
            end
            u = temp(1:2,:);
            x = temp(3:7,:);
            
            % Update the plot with the new results
            updatePlots(x,u,model,params);
            
        end
     end   
end

function updatePlots(x,u,model,params)    
% Deletes old data sets in the current plot and adds the new data sets 
% given by the arguments x, u and params to the plot.
% x: matrix consisting of a set of state column vectors
% u: matrix consisting of a set of input column vectors
% params: position of obstacle
% model: model struct required for the code generation of FORCESPRO

    gcf();
            
    % plot updated acceleration force
    subplot(5,2,8);          
    oldAcc = findobj(gca,'Color','blue');
    delete(oldAcc);
    stairs(u(1,:),'b-'); grid on; 

    % plot updated steering rate
    subplot(5,2,10)
    oldPhi = findobj(gca,'Color','blue');
    delete(oldPhi);
    stairs(rad2deg(u(2,:)),'b-'); grid on;

    % plot updated velocity
    subplot(5,2,2)
    oldVel = findobj(gca,'Color','blue');
    delete(oldVel);
    plot(x(3,:),'b-'); grid on;

    % plot updated heading angle
    subplot(5,2,4) 
    oldTheta = findobj(gca,'Color','blue');
    delete(oldTheta)
    plot(rad2deg(x(4,:)),'b-'); grid on; 

    % plot updated steering angle 
    subplot(5,2,6)
    oldDelta = findobj(gca,'Color','blue');
    delete(oldDelta);
    plot(rad2deg(x(5,:)), 'b-'); grid on

    % plot updated trajectory
    subplot(5,2,[1,3,5,7,9])
    oldPath = findobj(gca,'Color','blue','LineStyle','-');
    delete(oldPath);
    plot(x(1,:),x(2,:),'b-'); hold on;
    oldObstacle = findobj(gca,'LineStyle','-.');
    delete(oldObstacle);
    rectangle('Position',[params(1)-...
        sqrt(model.hl(2)) params(2)-sqrt(model.hl(2)) ...
        2*sqrt(model.hl(2)) 2*sqrt(model.hl(2))],...
        'Curvature',[1 1],'EdgeColor','r','LineStyle','-.');
    legend('init pos','trajectory','Location','northwest');   
end

function plotAndMakeInteractive(x,u,model,params)
% Creates a plot, adds the data sets provided by the arguments x, u, and
% params to the plot and makes the plot interactive by connecting a
% callback function
% x: matrix consisting of a set of state column vectors
% u: matrix consisting of a set of input column vectors
% params: position of obstacle
% model: model struct required for the code generation of FORCESPRO

    figure(1); clf;

    % plot acceleration force
    subplot(5,2,8)
    title('Acceleration force'); hold on;
    plot([1 model.N], [model.ub(1) model.ub(1)]', 'r:');
    plot([1 model.N], [model.lb(1) model.lb(1)]', 'r:');
    stairs(u(1,:),'b-');

    % plot steering rate
    subplot(5,2,10);
    title('Steering rate'); hold on;
    plot([1 model.N], rad2deg([model.ub(2) model.ub(2)]'), 'r:');
    plot([1 model.N], rad2deg([model.lb(2) model.lb(2)]'), 'r:');
    stairs(rad2deg(u(2,:)),'b-');
    
    % plot velocity
    subplot(5,2,2)
    title('Velocity'); hold on;
    plot([1 model.N], [model.ub(5) model.ub(5)]', 'r:');
    plot([1 model.N], [model.lb(5) model.lb(5)]', 'r:');
    plot(x(3,:),'b-');

    % plot heading angle
    subplot(5,2,4)
    title('Heading angle'); hold on;
    xlim([1, model.N]);
    ylim([-90,180]);
    plot(rad2deg(x(4,:)),'b-');

    % plot steering angle
    subplot(5,2,6)
    title('Steering angle'); hold on;
    plot([1 model.N], rad2deg([model.ub(7) model.ub(7)])', 'r:');
    plot([1 model.N], rad2deg([model.lb(7) model.lb(7)])', 'r:');
    plot(rad2deg(x(5,:)),'b-');

    % plot trajectory in interactive window
    subplot(5,2,[1,3,7,9])
    plot(model.xinit(1),model.xinit(2),'bx','LineWidth',3); hold on;
    rectangle('Position',[-sqrt(model.hl(1)) -sqrt(model.hl(1)) ...
        2*sqrt(model.hl(1)) 2*sqrt(model.hl(1))],'Curvature',[1 1], ...
        'EdgeColor','r','LineStyle',':');
    rectangle('Position',[-sqrt(model.hu(1)) -sqrt(model.hu(1)) ...
        2*sqrt(model.hu(1)) 2*sqrt(model.hu(1))],'Curvature',[1 1], ...
        'EdgeColor','r','LineStyle',':');
    plot(x(1,:),x(2,:),'b-');
    rectangle('Position',[params(1,1)- sqrt(model.hl(2))...
        params(2,1)-sqrt(model.hl(2)) 2*sqrt(model.hl(2)) ...
        2*sqrt(model.hl(2))],'Curvature',[1 1],'EdgeColor','r',...
            'LineStyle','-.');
    legend('init pos','trajectory','Location','northwest');
    title(sprintf('Click into figure to place obstacle'));
    axis equal;
    xlim([model.lb(3), model.ub(3)]);
    ylim([model.lb(4),model.ub(4)]);
    xlabel('x-coordinate'); ylabel('y-coordinate');
    set(gcf,'WindowButtonDownFcn',{@calculatePath,model})
end
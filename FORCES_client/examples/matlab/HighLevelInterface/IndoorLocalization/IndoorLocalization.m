% This example shows how to implement a least-squares decoder for
% localization in 2D.
%
% Assume we have noisy distance measurements d_i (noise assumed to be
% zero mean and i.i.d.) from N anchors, i.e. d_i is a N-column vector.
% We want to estimate the position (x,y) of a target by solving for
% least-squares error
%
%    minimize \sum_i=1^N e_i^2
%  subject to  e_i == (xa-xhat)^2 + (ya-yhat)^2 - d_i^2
%
% where (xa,ya) are the known position of the anchors (xa,ya).
%
% (c) Embotech AG, Zurich, Switzerland, 2013-2021.
function positionEstimator

close all; clc;

global N xlimits ylimits xa ya noise

%% configure settings & parameters
N = 4;                              % number of anchors
xlimits = [0,10]; ylimits = [0,10]; % limits for x and y of world
xa = [0, 10, 0,   10]';             % x-positions for anchors
ya = [0, 0,  10,  10]';             % y-positions for anchors

noise = 0.1;  % noise standard deviation in meters

%% do not change code below
% (but do read it)

% some assertions
assert(length(xa)==N,'xa must have length %d', N);
assert(all(xa>=xlimits(1)),'xa out of world');
assert(all(ya>=ylimits(1)),'ya out of world');
assert(all(xa<=xlimits(2)),'xa out of world');
assert(all(ya<=ylimits(2)),'ya out of world');


%% Generate code for estimator
generateEstimator(N, xlimits, ylimits);

%% Plot & make interactive
msize = 8;
figure(1); clf;
plot(xa(1),ya(1),'kx'); hold on; plot(xa(1),ya(1),'bx'); plot(xa(1),ya(1),'ro');
plot(xa,ya,'kx','markersize',msize); plot(xa,ya,'ko','markersize',msize);
xlim(xlimits+[-1,1]); ylim(ylimits+[-1,1]);
title(sprintf('Click into figure to place target (noise level: %3.1f)', noise));
legend({'anchors','true position','estimated position'},'Location','NorthEastOutside');
set(gcf,'WindowButtonDownFcn',@estimatePosition)
axis equal
end


%% distance function
function d = distance(xa,xtrue,ya,ytrue)
d = sqrt((xa-xtrue).^2 + (ya-ytrue).^2);
end

%% Callback for plot
function [xhat,yhat]=estimatePosition(hObject,~)

global N xlimits ylimits xa ya noise
msize = 8;

% read in true position
pos=get(gca,'CurrentPoint');
xtrue=pos(1,1);
ytrue=pos(1,2);
disp(['You clicked X: ',num2str(xtrue),', Y: ',num2str(ytrue)]);
assert(xtrue <= xlimits(2),'xtrue out of world');
assert(xtrue >= xlimits(1),'xtrue out of world');
assert(ytrue <= ylimits(2),'ytrue out of world');
assert(ytrue >= ylimits(1),'ytrue out of world');
plot(xtrue,ytrue,'bx','markersize',msize);

% generate noisy measurements
d= distance(xa,xtrue,ya,ytrue) + noise*randn(N,1);

% feed problem data
problem.x0 = zeros(2,1);
problem.all_parameters = [xa; ya; d]; % fill the parameter (p)-vector

% solve!
[output,exitflag,info] = localizationDecoder(problem);
assert(exitflag==1,'some problem in solver'); % always test exitflag for success
xhat = output.x1(1);
yhat = output.x1(2);
esterr = norm([xhat;yhat]-[xtrue;ytrue]);

% plot
plot(xhat,yhat,'ro','markersize',msize);

% print
fprintf('Estimated   X: %6.4f, Y: %6.4f\n', xhat, yhat);
fprintf('This is an estimation error of %6.4f. Solvetime: %6.4f microsceonds.\n', esterr, info.solvetime*1E6);

end


%% This function generates the estimator
function generateEstimator(numberOfAnchors,xlimits,ylimits)
% Generates 2D decoding code for localization using FORCESPRO NLP
% na: number of anchors

global na
na = numberOfAnchors;

%% NLP problem definition
% no need to change anything below
model.N = 1;      % number of distance measurements
model.nvar = 2;   % number of variables (use 3 if 3D)
model.npar = numberOfAnchors*3; % number of parameters: coordinates of anchors in 2D, plus measurements
model.objective = @objective;
model.lb = [xlimits(1) ylimits(1)]; % lower bounds on (x,y)
model.ub = [xlimits(2) ylimits(2)]; % upper bounds on (x,y)

%% codesettings
codesettings = getOptions('localizationDecoder');
codesettings.printlevel = 0; % set to 2 to see some prints
% codesettings.server = 'http://winner10:2470';
codesettings.maxit = 50; % maximum number of iterations
codesettings.nlp.ad_tool = 'casadi';
%codesettings.nlp.ad_tool = 'symbolic-math-tbx';

%% generate code
FORCES_NLP(model, codesettings);

end

%% This function implements the objective
% We assume that the parameter vector p is ordered as follows:
% p(1:na)        - x-coordinates of the anchors
% p(na+(1:na))   - y-coordinates of the anchors
% p(2*na+(1:na)) - distance measurements of the anchors 
function obj = objective( z,p )

global na
obj=0;
for i = 1:na
    obj = obj + ( (p(i)-z(1))^2 + (p(i+na)-z(2))^2 - p(i+2*na)^2 )^2;
end
end
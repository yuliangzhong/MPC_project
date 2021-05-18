function [Y, U] = simulateMpcForces(mpcObj, disPlant, simLength, xinit, yref, measOutId, coredata, statedata, onlinedata, uref)
    % Auxiliary function for simulating linear MPC using mpcmoveForces
    % Inputs
    %   mpcObj:         MPC object that contains constraints, weights,...
    %   disPlant:       discrete plant linear model used for simulation  
    %   simLength:      number of simulation steps
    %   xinit:          initial state
    %   yref:           output reference
    %   measOutId:      indices of measured outputs
    %   forcesProData:  structure contaning FORCESPRO model, state and
    %                   online information
    %   uref:           output reference
    % (c) Embotech AG, Zurich, Switzerland, 2019-2021.

    nx = size(disPlant.A, 1);
    nu = size(disPlant.B, 2);
    ny = size(disPlant.C, 1); 
    
    X = zeros(nx, simLength+1);
    U = zeros(nu, simLength);
    Y = zeros(ny, simLength);
    X(:, 1) = xinit;
    for t = 1:simLength
        Y(:, t) = disPlant.C * X(:,t) + disPlant.D * U(:, t);
        % Prepare inputs of mpcmoveForces
        if ~isempty(yref)
            % Output reference
            onlinedata.signals.ref = yref(t:min(t+mpcObj.PredictionHorizon-1,simLength),:);
        end
        if ~isempty(uref)
            % Input reference
            onlinedata.signals.mvTarget = uref(t:min(t+mpcObj.PredictionHorizon-1,simLength),:);
        end
        onlinedata.signals.ym = Y(measOutId, t);
        [mv, statedata, info] = mpcmoveForces(coredata, statedata, onlinedata);
        if info.ExitFlag < 0
            warning('Internal problem in FORCESPRO solver.');
        end
        U(:, t) = mv;
        X(:, t+1) = disPlant.A * X(:, t) + disPlant.B * U(:, t);
    end

end


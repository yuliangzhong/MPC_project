function [self, success] = optimizerFORCES( constraint,objective,codeoptions,parameters,solverOutputs,parameterNames,outputNames,mode )
%OPTIMIZERFORCES Generates a FORCES PRO solver from a YALMIP problem formulation
%
%   solver = OPTIMIZERFORCES(constraint,objective,codeoptions,parameters,solverOutputs)
%   generates a solver using FORCES PRO that solves the specified problem
%   for given parameter values and returns the value of the specified
%   outputs. The solver can be called  by indexing the returned OPTIMIZERFORCES
%   object with the parameter values. The parameter values can be passed in
%   a number of different ways:
%   
%       output = solver{param1, param2, ...} OR
%       output = solver{ {param1, param2, ...} } OR
%       output = solver(param1, param2, ...)
%   
%   This API is compatible with the YALMIP function OPTIMIZER. To obtain more
%   information on the solver, you can type 'help <solvername>' after the code
%   has been generated, where <solvername> is the name you give to the solver
%   via codeoptions.name.
%
%
%   solver = OPTIMIZERFORCES(constraint,objective,codeoptions,parameters,solverOutputs,parameterNames,outputNames)
%   same as above, but the generated code and all its associated help
%   files, the Simulink block etc. will carry the names provided in the
%   last two arguments for parameters and outputs, respectively. It is
%   strongly recommended to provide these names.
%
%
%   Example usage:
%
%       sdpvar x y p
%       options = getOptions('solver_name')
%     	solver = optimizerFORCES([x + y == p], x^2+y^2, options, p, {x,y}, {'p'}, {'x_optimal','y_optimal'})
%       solution = solver{5} % solve problem for p = 5
%
%
%   Inputs:
%       constraint:     constraints in YALMIP format, see e.g. OPTIMIZER
%       objective:      objective in YALMIP format, see e.g. OPTIMIZER
%       codeoptions:    options for FORCES PRO solver, see GETOPTIONS
%       parameters:     Single SDPVAR object or cell array of SDPVAR
%                       objects that should be considered a parameter
%       solverOutputs:  Single SDPVAR object or cell array of SDPVAR
%                       objects whose value(s) should be returned by the
%                       solver, can be a linear combination of decision
%                       variables and parameters
%       parameterNames: (optional) Cell array of strings with names for
%                       parameters that will, for example, be used in the
%                       Simulink block. If names are not specified, they
%                       will be auto-generated.
%       outputNames:    (optional) Cell array of strings with names for
%                       outputs that will, for example, be used in the
%                       Simulink block. If names are not specified, they
%                       will be auto-generated.
%       mode:           (optional) 'default' will generate a solver,
%                       'dump' will only return a solver object without 
%                       actually generating the solver,
%                       'dump_anonymized' does the same as 'dump' and 
%                       additionally applies some basic anonymization
%
%   Outputs:
%       solver:         reference to OPTIMIZERFORCES object. Use this to
%                       call solver. Example:
%                           solver = optimizerFORCES(...);
%                           x = solver{paramValues};
%       success:        1 if solver generation was successful, 0 otherwise
%
%
%   Getting details on solver:
%
%       help <solvername> % get more details on the generated solver
%
% See also SDPVAR OPTIMIZER
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

    if (nargin < 1)
        constraint = [];
    end
    if (nargin < 2)
        objective = [];
    end
    if (nargin < 3)
        codeoptions = [];
    end
    if (nargin < 4)
        parameters = [];
    end
    if (nargin < 5)
        solverOutputs = [];
    end
    if (nargin < 6)
        parameterNames = {};
    end
    if (nargin < 7)
        outputNames = {};
    end
    if (nargin < 8) || isempty(mode)
        mode = 'default';
    end

    self = setupOptimizerForcesClass( codeoptions,parameters,parameterNames,outputNames );
    
    % check for class constructor call
    if (nargin == 0) && (nargout <= 1)
        return;
    end

    disp('YALMIP-to-FORCES code generator')
    disp('-------------------------------')

    % Check if YALMIP is installed
    if ~exist('optimizer','file')
        error('YALMIP could not be found. Please make sure it is installed correctly.')
    end

    if (nargin >= 5)
        inputNames = {inputname(4),inputname(5)};
    else
        inputNames = {'',''};
    end
    [ self,solverOutputs ] = sanitizeInputData( self,solverOutputs, constraint,objective,inputNames );
    
    if strcmpi( mode,'dump_anonymized' )
        disp('Basic anonymization enabled.');
        self = anonymizeNames( self );
    end
    
    
    %% Call YALMIP and convert QP into FORCES format
    disp('This is Y2F (v0.1.19), the YALMIP interface of FORCES PRO.');
    disp('For more information visit https://github.com/embotech/y2f');
    fprintf('\nUsing YALMIP to convert problem into QP...')
    
    tic;
    [ qpData,internalmodel ] = getQpAndModelFromYALMIP( constraint,objective );
    yalmipTime = toc;
    fprintf('   [OK, %5.1f sec]\n', yalmipTime);

    qpData = sanitizeQpData( qpData );

    %% Assemble parameters & convert quadratic variables
    % Quadratic inequalities are not recognized by YALMIP
    % Information is stored in internalmodel
    fprintf('Extract parameters and quadratic inequalities from YALMIP model...')
    tic;
    [ self,qpData, Q,l,r,paramVars,yalmipParamMap ] = buildParamsAndQuadIneqs( self,qpData,internalmodel );
    extractStagesTime = toc;
    fprintf('   [OK, %5.1f sec]\n', extractStagesTime);

    %% Assemble stages
    fprintf('Assembling stages...')
    tic;
    self = assembleStages( self,qpData,solverOutputs, Q,l,r,paramVars,yalmipParamMap );
    assembleStagesTime = toc;
    fprintf('   [OK, %5.1f sec]\n', assembleStagesTime);

    %% Print stage sizes
    if self.numSolvers == 1
        fprintf('Found %u stages:\n', numel(self.stages{1}));
        printStageSizes(self.stages{1},'  ');
    else  % we have multiple solvers
        fprintf('The problem is separable. %u solvers are needed:\n', self.numSolvers);
        for i=1:self.numSolvers
            fprintf('    - Solver %u has %u stages:\n', i, numel(self.stages{i}));
            printStageSizes(self.stages{i},'        ');
        end
    end

    %% Adjust user-defined codeoptions
    % backing up user-defined codeoptions
    self.default_codeoptions = self.codeoptions;
    
    % set flag to let FORCES know that request came from Y2F
    self.default_codeoptions.interface = 'y2f';
        
    self.codeoptions = cell(1,self.numSolvers);
    for i=1:self.numSolvers
        self.codeoptions{i} = self.default_codeoptions;
        % new name for each solver
        self.codeoptions{i}.name = sprintf('internal_%s_%u',self.default_codeoptions.name,i);
        self.codeoptions{i}.nohash = 1; % added by AD to avoid problem - exeprimental
    end
    
    self.interfaceFunction = str2func(self.default_codeoptions.name);

    %% Generate solver using FORCESPRO (if requested)
    if strcmpi( mode,'dump' ) || strcmpi( mode,'dump_anonymized' )
        % only dump current controller
        success = 0;
        return;
    else
        disp('Generating solver using FORCESPRO...')
        success = buildSolver( self );
    end

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HELPER FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ self ] = setupOptimizerForcesClass( codeoptions,parameters,parameterNames,outputNames )
% Helper function to setup struct that is going to be converted into the optimizerFORCES class.

    self = struct();

    self.codeoptions = codeoptions;
    self.parameters = parameters;
    self.paramNames = parameterNames;
    self.outputNames = outputNames;
    
    self.qcqpParams = struct();
    self.stages = {};
    self.params = {};
    self.standardParamValues = {};
    self.forcesParamMap = {};
    self.outputFORCES = {};
    self.outputMap = [];
    self.outputBase = {};
    self.outputSize = {};
    self.lengthOutput = 0;
    self.numSolvers = 0;

    self.default_codeoptions = [];
    self.solverHasParams = [];
    self.solverIsBinary = [];
    self.solverVars = [];
    self.paramSizes = [];
    self.numParams = 0;
    self.outputIsCell = 1;
    self.interfaceFunction = [];
    
    self = class(self,'optimizerFORCES');
    
end


function [ self,solverOutputs ] = sanitizeInputData( self,solverOutputs, constraint,objective,inputNames )
% Helper function to make sure certain user input is given in the correct format.

    % We need all those arguments
    if isempty(constraint)
        error('Constraints not found');
    end
    if isempty(objective)
        error('Objective not found');
    end
    if isempty(self.codeoptions)
        error('Solver options not found');
    end
    if isempty(self.parameters)
        error('Parameter(s) not found');
    end
    if isempty(solverOutputs)
        error('Output(s) not found');
    end

    % Make valid solver name
    if ~verLessThan('matlab', '8.3')
        self.codeoptions.name = matlab.lang.makeValidName(self.codeoptions.name);
    else
        self.codeoptions.name = genvarname(self.codeoptions.name);
    end

    % We need parameters
    if isempty(self.parameters)
        error('FORCES PRO does not support problems without parameters.');
    end

    % Read parameter names if they were passed along
    if ~isempty(self.paramNames)
        if ~iscellstr(self.paramNames)
            error('parameterNames needs to be a cell array of strings.')
        end

        % Fix names (make them valid and unique)
        if ~verLessThan('matlab', '8.3')
            self.paramNames = matlab.lang.makeValidName(self.paramNames);
            self.paramNames = matlab.lang.makeUniqueStrings(self.paramNames);
        else
            self.paramNames = genvarname(self.paramNames);
        end
    elseif isa(solverOutputs,'sdpvar')
        % Single parameter supplied, we might get its name!
        name = inputNames{1};
        if ~isempty(name)
            self.paramNames = {name};
        else
            self.paramNames = {};
            warning('Y2F:noParameterNames',['No parameter names specified for solver. We recommend adding names for better code documentation. ' ...
            'For more info type ''help optimizerFORCES''.']);
        end
    else
        self.paramNames = {};
        warning('Y2F:noParameterNames',['No parameter names specified for solver. We recommend adding names for better code documentation. ' ...
            'For more info type ''help optimizerFORCES''.']);
    end

    % Read output names if they were passed along
    if ~isempty(self.outputNames)
        if ~iscellstr(self.outputNames)
            error('outputNames needs to be a cell array of strings.')
        end

        % Fix names (make them valid and unique)
        if ~verLessThan('matlab', '8.3')
            self.outputNames = matlab.lang.makeValidName(self.outputNames);
            self.outputNames = matlab.lang.makeUniqueStrings(self.outputNames);
        else
            self.outputNames = genvarname(self.outputNames);
        end
    elseif isa(solverOutputs,'sdpvar')
        % Single output supplied, we might get its name!
        name = inputNames{2};
        if ~isempty(name)
            self.outputNames = {name};
        else
            self.outputNames = {};
            warning('Y2F:noOutputNames',['No output names specified for solver. We recommend adding names for better code documentation. ' ...
            'For more info type ''help optimizerFORCES''.']);
        end
    else
        self.outputNames = {};
        warning('Y2F:noOutputNames',['No output names specified for solver. We recommend adding names for better code documentation. ' ...
            'For more info type ''help optimizerFORCES''.']);
    end

    % Create missing parameter names
    while numel(self.paramNames) < numel(self.parameters)
        self.paramNames{end+1} = sprintf('param%u', numel(self.paramNames)+1);
    end

    % We allow single parameters --> wrap them in a cell array
    if ~iscell(self.parameters) 
        self.parameters = {self.parameters}; % put single param into cell array
    end

    if ~iscell(solverOutputs)
        solverOutputs = {solverOutputs};
        self.outputIsCell = 0;
    end

    % Create missing parameter names
    while numel(self.outputNames) < numel(solverOutputs)
        self.outputNames{end+1} = sprintf('output%u', numel(self.outputNames)+1);
    end

end


function [ self ] = anonymizeNames( self )
% Helper function to replace solver, parameter, and output names by 
% generic ones.

    % parameter names
    for ii=1:length(self.paramNames)
        self.paramNames{ii} = ['param',num2str(ii)];
    end
    
    % output names
    for ii=1:length(self.outputNames)
        self.outputNames{ii} = ['output',num2str(ii)];
    end
    
    % solver name
    self.codeoptions.name = 'y2f_solver';

end


function [ qpData ] = sanitizeQpData( qpData )
% Helper function to perform sanity checks on QP data that
% throws errors or warnings if an issue is detected.

    % Check if matrices are numeric
    if ~isnumeric(qpData.H) || ~isnumeric(qpData.f)
        error('Y2F can only handle numeric inputs. There are non-numeric terms in the cost.')
    end
    if ~isnumeric(qpData.Aineq) || ~isnumeric(qpData.bineq)
        error('Y2F can only handle numeric inputs. There are non-numeric terms in the inequality contraints.')
    end
    if ~isnumeric(qpData.Aeq) || ~isnumeric(qpData.beq)
        error('Y2F can only handle numeric inputs. There are non-numeric terms in the equality contraints.')
    end
    if ~isnumeric(qpData.lb) || ~isnumeric(qpData.ub)
        error('Y2F can only handle numeric inputs. There are non-numeric terms in the bounds.')
    end

    % Check if matrices are real
    if ~isreal(qpData.H) || ~isreal(qpData.f)
        error('Y2F can only handle real inputs. There are complex terms in the cost.')
    end
    if ~isreal(qpData.Aineq) || ~isreal(qpData.bineq)
        error('Y2F can only handle real inputs. There are complex terms in the inequality contraints.')
    end
    if ~isreal(qpData.Aeq) || ~isreal(qpData.beq)
        error('Y2F can only handle real inputs. There are complex terms in the equality contraints.')
    end
    if ~isreal(qpData.lb) || ~isreal(qpData.ub)
        error('Y2F can only handle real inputs. There are complex terms in the bounds.')
    end

    % Check if matrices are doubles
    if ~isa(qpData.H,'double') || ~isa(qpData.f,'double')
        warning('Y2F:nonDoubleCost', 'Y2F can only handle inputs of type ''double''. The cost will be cast to ''double''.')
        qpData.H = double(qpData.H);
        qpData.f = double(qpData.f);
    end
    if ~isa(qpData.Aineq,'double') || ~isa(qpData.bineq,'double')
        warning('Y2F:nonDoubleInequality', 'Y2F can only handle inputs of type ''double''. The inequality constraints will be cast to ''double''.')
        qpData.Aineq = double(qpData.Aineq);
        qpData.bineq = double(qpData.bineq);
    end
    if ~isa(qpData.Aeq,'double') || ~isa(qpData.beq,'double')
        warning('Y2F:nonDoubleEquality', 'Y2F can only handle inputs of type ''double''. The equality constraints will be cast to ''double''.')
        qpData.Aeq = double(qpData.Aeq);
        qpData.beq = double(qpData.beq);
    end
    if ~isa(qpData.lb,'double') || ~isa(qpData.ub,'double')
        warning('Y2F:nonDoubleBounds', 'Y2F can only handle inputs of type ''double''. The bounds will be cast to ''double''.')
        qpData.lb = double(qpData.lb);
        qpData.ub = double(qpData.ub);
    end
    
end


function [ qpData,internalmodel ] = getQpAndModelFromYALMIP( constraint,objective )
% Helper function that uses YALMIP to create Qp from user's constraints
% and objective. Parameters and quadratic constraints are ignored for
% now and handled later on.

    % Call YALMIP's export
    % model contains Qp (in quadprog format)
    % internalmodel contains data that we need to recover parameters and
    % quadratic inequalities
    options = sdpsettings('solver','+quadprog','verbose',2);

    % Hack: Get YALMIP to return sparse matrices
    options.linprog.LargeScale = 'on';
    options.quadprog.LargeScale = 'on';

    [model,~,diagnostics,internalmodel] = export(constraint,objective,options,[],[],1);
    if ~isempty(diagnostics)
        disp(diagnostics)
        error('YALMIP was not able to convert the problem into a QCQp.')
    end

    % Get matrices from quadprog model
    qpData = struct();
    qpData.H = model.Q;
    qpData.f = model.c;
    qpData.Aineq = model.A;
    qpData.bineq = model.b;
    qpData.Aeq = model.Aeq;
    qpData.beq = model.beq;
    qpData.lb = model.lb;
    qpData.ub = model.ub;

    % YALMIP doesn't always recognize bounds as such (and makes them
    % inequalities) --> we have to convert them
    bounds_idx = []; % remember to delete these
    for i=1:size(qpData.Aineq,1)
        vars = find(qpData.Aineq(i,:)); % variables used in inequality
        if length(vars) == 1 && internalmodel.variabletype(vars) == 0 % a single linear variable
            % we can make a bound out of this
            bounds_idx(end+1) = i; %#ok<AGROW>
            if qpData.Aineq(i,vars) > 0 % upper bound
                qpData.ub(vars) = min(qpData.ub(vars),qpData.bineq(i)/qpData.Aineq(i,vars));
            else % < 0 --> lower bound
                qpData.lb(vars) = max(qpData.lb(vars),qpData.bineq(i)/qpData.Aineq(i,vars));
            end
        end
    end
    qpData.Aineq(bounds_idx,:) = [];
    qpData.bineq(bounds_idx) = [];

end


function [ self,qpData,Q,l,r,paramVars,yalmipParamMap ] = buildParamsAndQuadIneqs( self,qpData,internalmodel )
% Helper function that builds QCQp parameter list and recognises
% quadratic inequalities

    % Build empty additive QCQp param struct
    qcqpParams.H = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.f = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.Aineq = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.bineq = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.Aeq = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.beq = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.l = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.Q = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.r = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.lb = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.ub = struct('maps2index',{},'maps2origparam',{},...
        'maps2mat',{},'factor',{});
    qcqpParams.bidx = [];

    % Build empty quadratic constraints
    Q = {};
    l = [];
    r = [];

    % Find all YALMIP variable indices for parameters & build parameter map
    paramVars = []; % YALMIP variables that are parameters
    yalmipParamMap = zeros(2,0); % 1st row: index of matrix with values,
    % 2nd row: index of element inside matrix
    if ~isempty(self.parameters)
        self.paramSizes = zeros(numel(self.parameters),2);
        for i=1:numel(self.parameters)
            if ~isa(self.parameters{i}, 'sdpvar')
                error('Parameters must be a SDPVAR or a cell array of SDPVARs.');
            end

            % store size for code generation
            self.paramSizes(i,:) = size(self.parameters{i});

            % find YALMIP variables that make up parameter
            newParams = getvariables(self.parameters{i});
            paramVars = [paramVars newParams]; %#ok<AGROW>

            % find element inside matrix (given by user) that contains value of
            % parameter
            for p=newParams
                yalmipParamMap(:,end+1) = [i; find(getbasematrix(self.parameters{i},p),1)]; %#ok<AGROW>
            end
        end

        assert(length(paramVars) == size(yalmipParamMap,2));
        self.numParams = numel(self.parameters);
    end
    
    % erase YALMIP parameters after last use to allow dumping of 
    % optimizerFORCES class (note that sdpvars cannot be stored in a MAT file!)
    self.parameters = {};

    quadIneq = zeros(2,0); % data structure for keeping track of (parametric) quadratic inequalities
    % first row:     id of linear inequality
    % second row:    id of quad inequality

    % Go through all variables and classify them. Remove parameters and
    % interpret
    removeIdx = []; % don't forget to delete these
    for i=length(internalmodel.used_variables):-1:1 % go through all YALMIP variables
        var = recover(internalmodel.used_variables(i)); % get corresponding SDPVAR object

        % is this variables binary?
        binary = ismember(i,internalmodel.integer_variables);
        if binary
            qcqpParams.bidx(end+1) = i;

            if qpData.lb(i) ~= 0 || qpData.ub(i) ~= 1
                error('No integer variables other than binary supported.');
            end
        end

        % Is this variable a linear parameter?
        if is(var, 'linear') && ...
                any(paramVars == internalmodel.used_variables(i))

            % Parameters cannot be binary
            if binary
                error('Parameters cannot be binary.');
            end

            % index in parameter list (needed to recover value later on)
            p_idx = find(paramVars == internalmodel.used_variables(i),1);

            % Variable is parameter --> remove it
            removeIdx = [removeIdx i]; %#ok<AGROW>

            % Check if parameter appears in H --> put into f
            rows = find(qpData.H(:,i));
            rows = rows(rows ~= i); % param*param just adds a constant term to cost
            for row=rows'
                if ~any(paramVars == internalmodel.used_variables(row)) % param1*param2 just adds a constant term to cost
                    qcqpParams.f(end+1) = newAdditiveQcqpParam(row,p_idx,1,0.5*qpData.H(row,i));
                end
            end
            cols = find(qpData.H(i,:));
            cols = cols(cols ~= i); % param*param just adds a constant term to cost
            for col=cols
                if ~any(paramVars == internalmodel.used_variables(col)) % param1*param2 just adds a constant term to cost
                    qcqpParams.f(end+1) = newAdditiveQcqpParam(col,p_idx,1,0.5*qpData.H(i,col));
                end
            end

            % We can ignore it if param appears in f (just a constant term)

            % Check if parameter is used in Aineq --> add to bineq
            rows = find(qpData.Aineq(:,i));
            for row=rows'
                qcqpParams.bineq(end+1) = newAdditiveQcqpParam(row,p_idx,1,-qpData.Aineq(row,i));
            end

            % Check if parameter is used in Aeq --> add to beq
            rows = find(qpData.Aeq(:,i));
            for row=rows'
                qcqpParams.beq(end+1) = newAdditiveQcqpParam(row,p_idx,1,-qpData.Aeq(row,i));
            end

            % Check bounds
            if qpData.lb(i) ~= -Inf || qpData.ub(i) ~= Inf
                beep;
                warning('Y2F:parameterBounds','Bounds on parameters have no effect.')
            end

            % Quadratic constraints don't have to be checked

        elseif ~is(var, 'linear')

            % Nonlinear variables cannot be binary
            if binary
                error('Nonlinear variables cannot be binary.');
            end

            % Is variable bilinear/quadratic?
            if sum(internalmodel.monomtable(i,:)) == 2
                % Is variable the square of another variable?
                if any(internalmodel.monomtable(i,:)==2)
                    v_idx = find(internalmodel.monomtable(i,:)==2,1);
                    assert(length(v_idx)==1);
                    v = internalmodel.used_variables(v_idx);
                    if any(paramVars == v)
                        error('Parameters can only be used affinely.')
                    end

                    % Remove this pseudo-variable
                    removeIdx = [removeIdx i]; %#ok<AGROW>

                    % If it appears in the linear cost --> move it
                    if qpData.f(i) ~= 0
                        qpData.H(v_idx,v_idx) = qpData.H(v_idx,v_idx) + 2*qpData.f(i);
                    end

                    % Cannot appear in quadratic cost
                    if nnz(qpData.H(i,:)) > 0 || nnz(qpData.H(:,i)) > 0
                        error('Non-quadratic term appears in cost.')
                    end

                    % Cannot appear in equalities
                    if nnz(qpData.Aeq(:,i)) > 0
                        error('Quadratic equalities are not supported.')
                    end

                    % Check bounds
                    if qpData.lb(i) ~= -Inf || qpData.ub(i) ~= Inf
                        beep;
                        warning('Y2F:quadraticTermBounds','Bounds on quadratic terms have no effect.')
                    end

                    % Check if variable is used in Aineq --> put in quad. ineq.
                    rows = find(qpData.Aineq(:,i));
                    for row=rows'
                        [k,quadIneq,Q,l,r] = findOrCreateQuadraticInequality(qpData,row,quadIneq,Q,l,r);
                        Q{k}(v_idx,v_idx) = qpData.Aineq(row,i);
                    end

                    % Is it bilinearly dependent!
                    % (e.g. if C_i is a parameter)
                else
                    deps_idx = find(internalmodel.monomtable(i,:)==1);
                    assert(length(deps_idx) == 2);
                    deps = internalmodel.used_variables(deps_idx);

                    if all(ismember(deps, paramVars))
                        error('Parameters can only be used affinely.')
                    end

                    if ~any(ismember(deps, paramVars)) % no parameter
                        % Remove this pseudo-variable
                        removeIdx = [removeIdx i]; %#ok<AGROW>

                        % If it appears in the linear cost --> move it
                        if qpData.f(i) ~= 0
                            qpData.H(deps_idx(1),deps_idx(2)) = qpData.H(deps_idx(1),deps_idx(2)) + qpData.f(i);
                            qpData.H(deps_idx(2),deps_idx(1)) = qpData.H(deps_idx(2),deps_idx(1)) + qpData.f(i);
                        end

                        % Cannot appear in quadratic cost
                        if nnz(qpData.H(i,:)) > 0 || nnz(qpData.H(:,i)) > 0
                            error('Non-quadratic term appears in cost.')
                        end

                        % Cannot appear in equalities
                        if nnz(qpData.Aeq(:,i)) > 0
                            error('Bilinear equalities are not supported.')
                        end

                        % Check bounds
                        if qpData.lb(i) ~= -Inf || qpData.ub(i) ~= Inf
                            beep;
                            warning('Y2F:bilinearTermBounds','Bounds on bilinear terms have no effect.')
                        end

                        % Check if variable is used in Aineq --> put in quad. ineq.
                        rows = find(qpData.Aineq(:,i));
                        for row=rows'
                            [k,quadIneq,Q,l,r] = findOrCreateQuadraticInequality(qpData,row,quadIneq,Q,l,r);
                            Q{k}(deps_idx(1),deps_idx(2)) = 0.5*qpData.Aineq(row,i);
                            Q{k}(deps_idx(2),deps_idx(1)) = 0.5*qpData.Aineq(row,i);
                        end

                    else % a parameter is involed
                        if any(deps(1) == paramVars) && ...
                                ~any(deps(2) == paramVars) && ...
                                is(recover(deps(2)),'linear')
                            v = deps(2); % variable
                            v_idx = find(internalmodel.used_variables == v,1);
                            p = deps(1); % parameter
                            p_idx = find(paramVars == p,1);
                        elseif any(deps(2) == paramVars) && ...
                                ~any(deps(1) == paramVars) && ...
                                is(recover(deps(1)),'linear')
                            v = deps(1);
                            v_idx = find(internalmodel.used_variables == v,1);
                            p = deps(2);
                            p_idx = find(paramVars == p,1);
                        else % This shouldn't happen! Case is handled above
                            error('Internal error! Please contact support@embotech.ch')
                        end

                        % Variable is parameter --> remove it
                        removeIdx = [removeIdx i]; %#ok<AGROW>

                        % Does bilinear combo influence cost?
                        if qpData.f(i) ~= 0
                            qcqpParams.f(end+1) = newAdditiveQcqpParam(v_idx,p_idx,1,qpData.f(i));
                        end

                        if any(qpData.H(:,i)) || any(qpData.H(i,:))
                            error('Parameters can only be used affinely.')
                        end

                        % Check if parameter is used in Aineq
                        rows = find(qpData.Aineq(:,i));
                        for row=rows'
                            qcqpParams.Aineq(end+1) = newAdditiveQcqpParam(sub2ind(size(qpData.Aineq),row,v_idx),p_idx,1,qpData.Aineq(row,i));
                        end

                        % Check if parameter is used in Aeq
                        rows = find(qpData.Aeq(:,i));
                        for row=rows'
                            qcqpParams.Aeq(end+1) = newAdditiveQcqpParam(sub2ind(size(qpData.Aeq),row,v_idx),p_idx,1,qpData.Aeq(row,i));
                        end

                        % Check bounds
                        if qpData.lb(i) ~= -Inf || qpData.ub(i) ~= Inf
                            beep;
                            warning('Y2F:bilinearTermBounds','Bounds on bilinear terms have no effect.')
                        end
                    end
                end
            elseif sum(internalmodel.monomtable(i,:))==3
                % Decide which variable is the parameter
                % At the end v1 and v2 are real variables, p is the parameter
                temp_idx = find(internalmodel.monomtable(i,:));
                if length(temp_idx) == 2 % p*x^2
                    v1_idx = find(internalmodel.monomtable(i,:) == 2,1); % variable
                    v1 = internalmodel.used_variables(v1_idx);
                    v2_idx = v1_idx;
                    v2 = v1;
                    p = internalmodel.used_variables(find(internalmodel.monomtable(i,:) == 1,1));
                    p_idx = find(paramVars == p,1);
                    if isempty(p_idx) || any(paramVars == v1)
                        error('One of the non-quadratic terms cannot be interpreted.')
                    end
                elseif length(temp_idx) == 3 % p*x1*x2
                    temp_idx = find(internalmodel.monomtable(i,:) == 1);
                    temp_vars = internalmodel.used_variables(temp_idx); % variables & parameter
                    if ismember(temp_vars(1), paramVars) && ~any(ismember(temp_vars(2:3), paramVars))
                        v1_idx = temp_idx(2); % variables
                        v1 = temp_vars(2);
                        v2_idx = temp_idx(3);
                        v2 = temp_vars(3);
                        p = temp_vars(1);
                        p_idx = find(paramVars == p,1);
                    elseif ismember(temp_vars(2), paramVars) && ~any(ismember(temp_vars([1 3]), paramVars))
                        v1_idx = temp_idx(1); % variables
                        v1 = temp_vars(1);
                        v2_idx = temp_idx(3);
                        v2 = temp_vars(3);
                        p = temp_vars(2);
                        p_idx = find(paramVars == p,1);
                    elseif ismember(temp_vars(3), paramVars) && ~any(ismember(temp_vars(1:2), paramVars))
                        v1_idx = temp_idx(1); % variables
                        v1 = temp_vars(1);
                        v2_idx = temp_idx(2);
                        v2 = temp_vars(2);
                        p = temp_vars(3);
                        p_idx = find(paramVars == p,1);
                    else
                        error('One of the non-quadratic terms cannot be interpreted.')
                    end
                else
                    error('One of the non-quadratic terms cannot be interpreted.')
                end

                % Variable is parameter --> remove it
                removeIdx = [removeIdx i]; %#ok<AGROW>

                if qpData.f(i) ~= 0 % pseudo-variable affects cost --> param in H
                    qcqpParams.H(end+1) = newAdditiveQcqpParam(sub2ind(size(qpData.H),v1_idx,v2_idx),p_idx,1,qpData.f(i));
                    qcqpParams.H(end+1) = newAdditiveQcqpParam(sub2ind(size(qpData.H),v2_idx,v1_idx),p_idx,1,qpData.f(i));
                end

                % Cannot appear in quadratic cost
                if nnz(qpData.H(i,:)) > 0 || nnz(qpData.H(:,i)) > 0
                    error('Non-quadratic term appears in cost.')
                end

                % Cannot appear in equalities
                if nnz(qpData.Aeq(:,i)) > 0
                    error('Nonlinear equalities are not allowed.')
                end

                % Check bounds
                if qpData.lb(i) ~= -Inf || qpData.ub(i) ~= Inf
                    beep;
                    warning('Y2F:nonlinearTermBounds','Bounds on nonlinear terms have no effect.')
                end

                % Check inequalities
                rows = find(qpData.Aineq(:,i));
                for row=rows'
                    [k,quadIneq,Q,l,r] = findOrCreateQuadraticInequality(qpData,row,quadIneq,Q,l,r);
                    if v1_idx ~= v2_idx % make sure Q is symmetric
                        qcqpParams.Q(end+1) = newAdditiveQcqpParam(sub2ind(size(Q{k}),v1_idx,v2_idx),p_idx,k,0.5*qpData.Aineq(row,i));
                        qcqpParams.Q(end+1) = newAdditiveQcqpParam(sub2ind(size(Q{k}),v2_idx,v1_idx),p_idx,k,0.5*qpData.Aineq(row,i));
                    else
                        qcqpParams.Q(end+1) = newAdditiveQcqpParam(sub2ind(size(Q{k}),v1_idx,v1_idx),p_idx,k,qpData.Aineq(row,i));
                    end
                end
            else
                error('One of the non-quadratic terms cannot be interpreted.')
            end
        end
    end

    % Before removing params, every real state can be recovered
    self.solverVars = internalmodel.used_variables;
    self.solverVars(removeIdx) = [];

    % Compute shifts of variables (to adjust parameters)
    shift = zeros(1,length(internalmodel.used_variables));
    for i=1:length(shift)
        shift(i) = nnz(removeIdx <= i);
    end

    % Remove parameter indices from equations
    qpData.H(removeIdx,:) = [];
    qpData.H(:,removeIdx) = [];
    qpData.f(removeIdx) = [];
    qpData.Aineq(:,removeIdx) = [];
    qpData.Aeq(:,removeIdx) = [];
    qpData.lb(removeIdx) = [];
    qpData.ub(removeIdx) = [];
    if ~isempty(Q)
        for k=1:numel(Q)
            Q{k}(removeIdx,:) = []; %#ok<AGROW>
            Q{k}(:,removeIdx) = []; %#ok<AGROW>
        end
        l(removeIdx,:) = [];
    end

    % Shift indices of parameters
    for i=1:numel(qcqpParams.H)
        [row,col] = ind2sub(size(qpData.H)+length(removeIdx),qcqpParams.H(i).maps2index);
        qcqpParams.H(i).maps2index = sub2ind(size(qpData.H),row-shift(row),col-shift(col));
    end
    for i=1:numel(qcqpParams.f)
        qcqpParams.f(i).maps2index = qcqpParams.f(i).maps2index - shift(qcqpParams.f(i).maps2index);
    end
    for i=1:numel(qcqpParams.Aineq)
        [row,col] = ind2sub(size(qpData.Aineq)+[0 length(removeIdx)],qcqpParams.Aineq(i).maps2index);
        qcqpParams.Aineq(i).maps2index = sub2ind(size(qpData.Aineq),row,col-shift(col));
    end
    for i=1:numel(qcqpParams.Aeq)
        [row,col] = ind2sub(size(qpData.Aeq)+[0 length(removeIdx)],qcqpParams.Aeq(i).maps2index);
        qcqpParams.Aeq(i).maps2index = sub2ind(size(qpData.Aeq),row,col-shift(col));
    end
    for i=1:numel(qcqpParams.lb)
        qcqpParams.lb(i).maps2index = qcqpParams.lb(i).maps2index - shift(qcqpParams.lb(i).maps2index);
    end
    for i=1:numel(qcqpParams.ub)
        qcqpParams.ub(i).maps2index = qcqpParams.ub(i).maps2index - shift(qcqpParams.ub(i).maps2index);
    end
    for i=1:numel(qcqpParams.Q)
        [row,col] = ind2sub(size(Q{qcqpParams.Q(i).maps2mat})+length(removeIdx),qcqpParams.Q(i).maps2index);
        qcqpParams.Q(i).maps2index = sub2ind(size(Q{qcqpParams.Q(i).maps2mat}),row,col-shift(col));
    end
    for i=1:numel(qcqpParams.l)
        qcqpParams.l(i).maps2index = qcqpParams.l(i).maps2index - shift(qcqpParams.l(i).maps2index);
    end

    % Shift & sort binary variables
    qcqpParams.bidx = sort(qcqpParams.bidx - shift(qcqpParams.bidx));

    % Finish converting linear inequalities to quadratic ones
    for i=1:size(quadIneq,2)
        row = quadIneq(1,i);
        k = quadIneq(2,i);

        r(k) = r(k) + qpData.bineq(row); %#ok<AGROW>
        l(:,k) = l(:,k) + qpData.Aineq(row,:)'; %#ok<AGROW>

        % Convert bineq params
        relevantParams = findRelevantParams(row, 1, size(qpData.bineq), qcqpParams.bineq);
        for j=fliplr(relevantParams) % sort descending
            qcqpParams.r(end+1) = newAdditiveQcqpParam(k,qcqpParams.bineq(j).maps2origparam, ...
                1, qcqpParams.bineq(j).factor);
            qcqpParams.bineq(j) = [];
        end

        % Convert Aineq params
        relevantParams = findRelevantParams(row, 1:size(qpData.Aineq,2), size(qpData.Aineq), qcqpParams.Aineq);
        for j=fliplr(relevantParams) % sort descending
            [~,col] = ind2sub(size(qpData.Aineq), qcqpParams.Aineq(j).maps2index);
            qcqpParams.l(end+1) = newAdditiveQcqpParam(col,qcqpParams.Aineq(j).maps2origparam, ...
                k, qcqpParams.Aineq(j).factor);
            qcqpParams.Aineq(j) = [];
        end
    end
    
    % Compute shift in rows for linear inequalites
    shift = zeros(1,length(qpData.bineq));
    for i=1:length(shift)
        shift(i) = nnz(quadIneq(1,:) <= i);
    end
    
    % Delete linear inequalities
    qpData.Aineq(quadIneq(1,:),:) = [];
    qpData.bineq(quadIneq(1,:)) = [];
    
    % Fix parameters
    for i=1:numel(qcqpParams.Aineq)
        [row,col] = ind2sub(size(qpData.Aineq)+[length(quadIneq(1,:)) 0],qcqpParams.Aineq(i).maps2index);
        qcqpParams.Aineq(i).maps2index = sub2ind(size(qpData.Aineq),row-shift(row),col);
    end
    for i=1:numel(qcqpParams.bineq)
        qcqpParams.bineq(i).maps2index = qcqpParams.bineq(i).maps2index - shift(qcqpParams.bineq(i).maps2index);
    end

    % Convert inequalities to bounds - Round 2
    % This is only necessary if we have parameters that affect bounds
    bounds_idx = [];
    for i=1:size(qpData.Aineq,1)
        vars = find(qpData.Aineq(i,:)); % variables used in inequality
        if length(vars) == 1 && is(recover(self.solverVars(vars)),'linear')
            relevantParams = findRelevantParams(i, 1:size(qpData.Aineq,2), size(qpData.Aineq), qcqpParams.Aineq);
            if isempty(relevantParams) % No parameters affecting LHS of inequality
                if qpData.Aineq(i,vars) > 0 && qpData.ub(vars) == Inf % No bounds so far
                    % we can make a bound out of this
                    bounds_idx(end+1) = i; %#ok<AGROW>
                    qpData.ub(vars) = qpData.bineq(i)/qpData.Aineq(i,vars);

                    % Convert bineq params
                    relevantParams = findRelevantParams(i, 1, size(qpData.bineq), qcqpParams.bineq);
                    for j=fliplr(relevantParams) % sort descending
                        qcqpParams.ub(end+1) = newAdditiveQcqpParam(vars,qcqpParams.bineq(j).maps2origparam, ...
                            1, qcqpParams.bineq(j).factor/qpData.Aineq(i,vars));
                        qcqpParams.bineq(j) = [];
                    end
                elseif qpData.Aineq(i,vars) < 0 && qpData.lb(vars) == -Inf % No bounds so far
                    % we can make a bound out of this
                    bounds_idx(end+1) = i; %#ok<AGROW>
                    qpData.lb(vars) = qpData.bineq(i)/qpData.Aineq(i,vars);

                    % Convert bineq params
                    relevantParams = findRelevantParams(i, 1, size(qpData.bineq), qcqpParams.bineq);
                    for j=fliplr(relevantParams) % sort descending
                        qcqpParams.lb(end+1) = newAdditiveQcqpParam(vars,qcqpParams.bineq(j).maps2origparam, ...
                            1, qcqpParams.bineq(j).factor/qpData.Aineq(i,vars));
                        qcqpParams.bineq(j) = [];
                    end
                end
            end
        end
    end
    
    % Compute shift in rows for linear inequalites
    shift = zeros(1,length(qpData.bineq));
    for i=1:length(shift)
        shift(i) = nnz(bounds_idx <= i);
    end
    
    % Delete rows
    qpData.Aineq(bounds_idx,:) = [];
    qpData.bineq(bounds_idx) = [];
    
    % Fix parameters
    for i=1:numel(qcqpParams.Aineq)
        [row,col] = ind2sub(size(qpData.Aineq)+[length(bounds_idx) 0],qcqpParams.Aineq(i).maps2index);
        qcqpParams.Aineq(i).maps2index = sub2ind(size(qpData.Aineq),row-shift(row),col);
    end
    for i=1:numel(qcqpParams.bineq)
        qcqpParams.bineq(i).maps2index = qcqpParams.bineq(i).maps2index - shift(qcqpParams.bineq(i).maps2index);
    end
    
    self.qcqpParams = qcqpParams;
    
end


function [ k,quadIneq,Q,l,r ] = findOrCreateQuadraticInequality( qpData,rowIdx,quadIneq,Q,l,r )
% Helper function to find/create quadratic inequalities
%   Input:
%       rowIdx      index of linear inequality (row in Aineq)
%       quadIneq    data structure containing information about quadratic
%                   inequalities
%                   first row:     id of linear inequality
%                   second row:    id of quad inequality
%       Q,l,r       quadratic inequalities
%
%   Output:
%       k           index of quadratic inequality (as in Q{k} = ...)
%       updated quadIneq, Q, l, and r
    id = find(quadIneq(1,:) == rowIdx);
    if ~isempty(id)
        assert(length(id) == 1);
        k = quadIneq(2,id);
    else
        Q{end+1} = spalloc(size(qpData.H,1),size(qpData.H,2),0);
        l(:,end+1) = spalloc(size(qpData.H,1),1,0);
        r(end+1) = 0;
        k = length(r);
        quadIneq(:,end+1) = [rowIdx;k];
    end
    
end


function [ self ] = assembleStages( self,qpData,solverOutputs, Q,l,r,paramVars,yalmipParamMap )
% Helper function to setup stages that are used to generate FORCESPRO solvers.

    % Construct matrices where parametric elements are == 1
    % This is necessary to build graph and recognise infeasible problems
    H_temp = qpData.H;
    H_temp([self.qcqpParams.H.maps2index]) = 1;
    Aineq_temp = qpData.Aineq;
    Aineq_temp([self.qcqpParams.Aineq.maps2index]) = 1;
    Aeq_temp = qpData.Aeq;
    Aeq_temp([self.qcqpParams.Aeq.maps2index]) = 1;
    Q_temp = Q;
    for i=1:numel(self.qcqpParams.Q)
        Q_temp{self.qcqpParams.Q(i).maps2mat}(self.qcqpParams.Q(i).maps2index) = 1;
    end
    l_temp = l;
    for i=1:numel(self.qcqpParams.l)
        l_temp(self.qcqpParams.l(i).maps2index,self.qcqpParams.l(i).maps2mat) = 1;
    end

    %% Warn the user if the problem is/might be infeasible
    checkQcqpForInfeasibility( self.qcqpParams,qpData.H,H_temp,Q,Q_temp,qpData.lb,qpData.ub );

    %% Generate standard stages
    % Construct (potentially multiple) path graphs from Qp

    % Compute decision variable indices that are needed in output
    % (only relevant for separable problems)
    outputIdx = [];
    for i=1:numel(solverOutputs)
        outputVars = getvariables(solverOutputs{i});
        outputIdx = [outputIdx find(ismember(self.solverVars, outputVars))]; %#ok<AGROW>
    end

    graphComponents = pathGraphsFromQcqp( H_temp,Aineq_temp,Aeq_temp,Q_temp,l_temp );
    [graphComponents, self.stages,self.params,self.standardParamValues,self.forcesParamMap] = ...
        stagesFromPathGraphs( graphComponents,qpData.H,qpData.f,qpData.Aineq,qpData.bineq,qpData.Aeq,qpData.beq,l,Q,r,qpData.lb,qpData.ub,self.qcqpParams,yalmipParamMap,outputIdx );
    
    self.numSolvers = numel(self.stages);
    
    %% Assemble the rest of the FORCES parameters
    % Fake a parameter for each solver if there are none (we need one for FORCES)
    self.solverHasParams = zeros(1,self.numSolvers);
    for i=1:self.numSolvers
        if isempty(self.params{i})
            self.params{i}(1) = newParam('p',1,'cost.f');
            self.standardParamValues{i} = self.stages{i}(1).cost.f;
            self.stages{i}(1).cost.f = [];
        else % count params
            self.solverHasParams(i) = 1;
        end
    end

    % Mark solvers that contain binary variables
    self.solverIsBinary = zeros(1,self.numSolvers);
    if ~isempty(self.qcqpParams.bidx) 
        for i=1:self.numSolvers
            if any(ismember(cell2mat(graphComponents{i}.vertices),self.qcqpParams.bidx))
                self.solverIsBinary(i) = 1;
            end
        end
    end

    % Assemble outputs
    self = buildOutput( self, solverOutputs,graphComponents,paramVars,yalmipParamMap );

end


function [] = checkQcqpForInfeasibility( qcqpParams,H,H_temp,Q,Q_temp,lb,ub )
% Helper function that checks if the QCQP might be infeasible and warns
% the user

    % Check if cost matrix is positive semi-definite
    if isempty(qcqpParams.H) && (~issymmetric(H) || ~all(eig(H) >= -1e-7))
        error('Hessian is not positive semi-definite.')
    end

    if ~issymmetric(H_temp)
        beep;
        warning('Y2F:nonsymmetricHessian','Hessian is not symmetric.')
    end

    % Check if bounds make sense
    if isempty(qcqpParams.lb) && isempty(qcqpParams.ub) && any(lb > ub)
        error('Bounds are infeasible.')
    end

    % Check if any quadratic constraint matrix is indefinite
    for k=1:numel(Q_temp)
        if ~issymmetric(Q_temp{k})
            beep;
            warning('Y2F:nonsymmetricQuadraticConstraint', ...
                'One of the quadratic constraint matrices is not symmetric.')
        end

        if isempty(qcqpParams.Q) && any(eig(Q{k}) < -1e-7) && any(eig(Q{k}) > -1e-7)
            beep;
            warning('Y2F:indefiniteQuadraticConstraint', ...
                'One of the quadratic constraint matrices is indefinite.')
        end
    end

    % Create warning if binary variables are used
    if ~isempty(qcqpParams.bidx)
        while true
            beep;
            % MATLAB displays anything between [\8 ... ]\8 in orange font
            in = input(['[' 8 'YALMIP created a mixed integer problem. This could slow down the generated solver.\nDo you want to continue [y/n]? ]' 8],'s');
            if strcmpi(in,'y')
                break
            elseif strcmpi(in,'n')
                error('Please reformulate your problem or contact support at support@embotech.com.');
            end
        end
    end
    
end


function [ self ] = buildOutput( self,solverOutputs,graphComponents,paramVars,yalmipParamMap )
% Helper function that builds the output struct required for the FORCES
% solver(s), an outputMap that allows to recover the wantend output
% values from the solver output, an outputParamTable that allows the
% usage of parameters in outputs

    self.outputFORCES = {};
    % we need to know which output to get from which solver
    self.outputMap = zeros(3,0); % 1st row: variable type (1=decision variable,2=parameter)
    % 2nd row: index of solver/index of parameter value
    % 3rd row: index of output/index of element inside value matrix

    o = ones(self.numSolvers,1); % counter variable outputs
    p = 1; % counter parameters
    k = 1; % counter total number of outputs
    for i=1:numel(solverOutputs)
        outputVars = getvariables(solverOutputs{i});
        self.outputBase{i} = full(getbase(solverOutputs{i}));
        self.outputSize{i} = size(solverOutputs{i});
        for j=1:length(outputVars)
            idx = find(self.solverVars == outputVars(j),1);
            if length(idx) == 1
                [stage, state, component] = findVariableIndex(graphComponents,idx);
                self.outputFORCES{component}(o(component)) = newOutput(sprintf('o_%u',o(component)), stage, state);
                self.outputMap(:,end+1) = [1; component; o(component)];
                o(component) = o(component) + 1;
            else
                idx = find(paramVars == outputVars(j),1);
                if length(idx) == 1
                    self.outputMap(:,end+1) = [2; yalmipParamMap(:,idx)];
                    p = p+1;
                else
                    error('Output is not valid. Only linear combinations of optimization variables and parameters are allowed.')
                end
            end
            k = k + 1;
        end
    end
    self.lengthOutput = k-1;
    
end


function [] = printStageSizes(stages, indentation)
% Helper function for printing number of variables, equalities and inequalities

    % Compute width of fields
    maxFieldValue = max(abs([length(stages) ...
        arrayfun(@(x) x.dims.n, stages) ...
        arrayfun(@(x) x.dims.r, stages) ...
        arrayfun(@(x) x.dims.l, stages) ...
        arrayfun(@(x) x.dims.u, stages) ...
        arrayfun(@(x) x.dims.p, stages) ...
        arrayfun(@(x) x.dims.q, stages)]));
    maxFieldWidth = max(1, ceil(log10(maxFieldValue+1)));

    % print header
    fprintf('%s  stage                    ', indentation);
    for k = 1:length(stages)
        fprintf('%*d  ', maxFieldWidth, k);
    end
    fprintf('\n');

    nameColWidth = 27;
    fprintf([indentation repmat('-', 1, nameColWidth + length(stages)*(2+maxFieldWidth)) '\n']);

    % print size of variables
    fprintf('%s  # variables:             ', indentation);
    for k = 1:length(stages)
        fprintf('%*d  ', maxFieldWidth, stages(k).dims.n);
    end
    fprintf('\n');

    % print size of equality constraints
    fprintf('%s  # equality constraints:  ', indentation);
    for k = 1:length(stages)
        fprintf('%*d  ', maxFieldWidth, stages(k).dims.r);
    end
    fprintf('\n');

    % print size of lower bounds
    fprintf('%s  # lower bounds:          ', indentation);
    for k = 1:length(stages)
        fprintf('%*d  ', maxFieldWidth, stages(k).dims.l);
    end
    fprintf('\n');

    % print size of upper bounds
    fprintf('%s  # upper bounds:          ', indentation);
    for k = 1:length(stages)
        fprintf('%*d  ', maxFieldWidth, stages(k).dims.u);
    end
    fprintf('\n');

    % size of polytopic constraints
    fprintf('%s  # polytopic constraints: ', indentation);
    for k = 1:length(stages)
        fprintf('%*d  ', maxFieldWidth, stages(k).dims.p);
    end
    fprintf('\n');

    % size of quadratic constraints
    fprintf('%s  # quadratic constraints: ', indentation);
    for k = 1:length(stages)
        fprintf('%*d  ', maxFieldWidth, stages(k).dims.q);
    end
    fprintf('\n');

end

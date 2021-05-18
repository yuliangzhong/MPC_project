function [stages, params, standardParamValues, forcesParamMap] = generateStagesFromGraph( G, H,f,Aineq,bineq,Aeq,beq,l,Q,r,lb,ub,qcqpParams,yalmipParamMap )
%GENERATESTAGESFROMGRAPH Generates stages from path graph containing all variables of the problem
% qcqpParams will be modified to contain a link to the params they are linked to
%   Input:
%       yalmipParamMap:         Map that maps YALMIP parameters to value matrices
%                                   1st row: index of matrix with values,
%                                   2nd row: index of element inside matrix
%       
%   Output:
%       stages:                 FORCES stages
%       params:                 FORCES parameters
%       standardParamValues:    "standard" values for FORCES parameters. 
%                               Not every element of the matrix has be 
%                               covered by a QCQP parameter. Param values
%                               get added to this value.
%       forcesParamMap:         Map that maps FORCES parameters to parameter value matrices
%       	.(param_name)(1,.):   index of element that's affected
%           .(param_name)(2,.):   factor by which value gets multiplied
%           .(param_name)(3,.):   matrix that contains value
%           .(param_name)(4,.):   points to element inside value matrix
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

SPARSITY_THRESHOLD = 0.2;

if ~checkIfGraphIsPathGraph(G)
    error('Path is not a path graph. Other structures are not supported.')
end

% Find all variable names
last_idx = G.vertices{1};
all_idx = sort(cell2mat(G.vertices));

% Prepare stages
stages = MultistageProblem(G.n);
params = struct('name',{},'maps2stage',{},'maps2data',{},'type',{},...
    'maps2mat',{},'structure',{},'variables',{}); % empty param struct
standardParamValues = struct;
p = 1;
if nargout >= 4
    forcesParamMap = struct;
end

% Construct matrices where parametric elements are == 1
H_temp = H;
H_temp([qcqpParams.H.maps2index]) = 1;
Aineq_temp = Aineq;
Aineq_temp([qcqpParams.Aineq.maps2index]) = 1;
Aeq_temp = Aeq;
Aeq_temp([qcqpParams.Aeq.maps2index]) = 1;
Q_temp = Q;
for i=1:numel(qcqpParams.Q)
    Q_temp{qcqpParams.Q(i).maps2mat}(qcqpParams.Q(i).maps2index) = 1;
end
l_temp = l;
for i=1:numel(qcqpParams.l)
    l_temp(qcqpParams.l(i).maps2index,qcqpParams.l(i).maps2mat) = 1;
end

% Assemble linear indices of all parameters (for faster checking afterwards)
qcqpParamIndices.H = [qcqpParams.H.maps2index];
qcqpParamIndices.f = [qcqpParams.f.maps2index];
qcqpParamIndices.Aineq = [qcqpParams.Aineq.maps2index];
qcqpParamIndices.bineq = [qcqpParams.bineq.maps2index];
qcqpParamIndices.Aeq = [qcqpParams.Aeq.maps2index];
qcqpParamIndices.beq = [qcqpParams.beq.maps2index];
qcqpParamIndices.l = [qcqpParams.l.maps2index];
qcqpParamIndices.Q = [qcqpParams.Q.maps2index];
qcqpParamIndices.r = [qcqpParams.r.maps2index];
qcqpParamIndices.lb = [qcqpParams.lb.maps2index];
qcqpParamIndices.ub = [qcqpParams.ub.maps2index];

% Go through all stages (and create them)
barwidth = 30;
%status = y2f_progressbar([],0,G.n,barwidth);
status = [];
createParamTime = 0;
for i=1:G.n
    idx = G.vertices{i}; % sorting just to make sure code below works
    stages(i).dims.n = length(idx); % length of stage variable zi 
    stages(i).cost.H = full(H(idx,idx)); % Hessian
    stages(i).cost.f = full(f(idx)); % linear term
    
    % Is H a parameter?
    local_H_temp = H_temp(idx,idx);
    if all(local_H_temp(~eye(length(idx))) == 0) % is H diagonal?
        tic;
        param = findRelevantQcqpParamsAndCreateDiagonalForcesParameter( ...
                i,idx,size(H),qcqpParams.H,qcqpParamIndices.H,'cost.H');
        if ~isempty(param)
            standardParamValues.(param.name) = stages(i).cost.H(1:(length(idx)+1):end);
            stages(i).cost.H = [];
        end
        createParamTime = createParamTime + toc;
    else % H not diagonal --> standard function
        tic;
        findRelevantQcqpParamsAndCreateForcesParameter( ...
                i,idx,idx,size(H),qcqpParams.H,qcqpParamIndices.H,'cost.H');
        createParamTime = createParamTime + toc;
    end

    % Is f a parameter?
    tic;
    findRelevantQcqpParamsAndCreateForcesParameter( ...
            i,idx,1,size(f),qcqpParams.f,qcqpParamIndices.f,'cost.f');
    createParamTime = createParamTime + toc;
    
    % Select relevant equalities and set the constraints
    eq_idx = find(sum(Aeq_temp(:,idx)~=0, 2))';
    eq_idx = eq_idx(sum(Aeq_temp(eq_idx,setdiff(all_idx,[last_idx idx]))~=0,2)==0);
    eq_idx = sort(eq_idx);
    if i == 1
        d1IsSet = ~isempty(Aeq_temp(eq_idx,idx));
    end
    if d1IsSet
        if i > 1
            stages(i-1).eq.C = full(Aeq(eq_idx,last_idx));
        end
        stages(i).dims.r = length(eq_idx); % number of equality constraints
        stages(i).eq.c = full(beq(eq_idx));
        stages(i).eq.D = full(Aeq(eq_idx,idx));
    else
        if i > 1
            stages(i-1).dims.r = length(eq_idx); % number of equality constraints
            stages(i-1).eq.C = full(Aeq(eq_idx,last_idx));
            stages(i-1).eq.c = full(beq(eq_idx));
        end
        stages(i).eq.D = full(Aeq(eq_idx,idx));
    end
    
    % Create parameters only if there are equations
    if ~isempty(eq_idx)
        % Is C a parameter?
        if i > 1
            tic;
            findRelevantQcqpParamsAndCreateForcesParameter( ...
                    i-1,eq_idx,last_idx,size(Aeq),qcqpParams.Aeq,qcqpParamIndices.Aeq,'eq.C');
            createParamTime = createParamTime + toc;
        end

        % Is D a parameter?
        tic;
        findRelevantQcqpParamsAndCreateForcesParameter( ...
                i,eq_idx,idx,size(Aeq),qcqpParams.Aeq,qcqpParamIndices.Aeq,'eq.D');
        createParamTime = createParamTime + toc;

        % Is c a parameter?
        if d1IsSet
            tic;
            findRelevantQcqpParamsAndCreateForcesParameter( ...
                    i,eq_idx,1,size(beq),qcqpParams.beq,qcqpParamIndices.beq,'eq.c');
            createParamTime = createParamTime + toc;
        else
            tic;
            findRelevantQcqpParamsAndCreateForcesParameter( ...
                    i-1,eq_idx,1,size(beq),qcqpParams.beq,qcqpParamIndices.beq,'eq.c');
            createParamTime = createParamTime + toc;
        end
    end
    
    % Lower bounds
    temp_lb = lb(idx);
    stages(i).dims.l = sum(temp_lb ~= -Inf); % number of lower bounds 
    stages(i).ineq.b.lbidx = find(temp_lb ~= -Inf)'; % index vector for lower bounds
    stages(i).ineq.b.lb = full(temp_lb(temp_lb ~= -Inf));    % lower bounds
    
    % Create parameter only if there are bounds
    if ~isempty(stages(i).ineq.b.lbidx)
        % Is lb a parameter?
        tic;
        findRelevantQcqpParamsAndCreateForcesParameter( ...
                i,idx(stages(i).ineq.b.lbidx),1,size(lb),qcqpParams.lb,qcqpParamIndices.lb,'ineq.b.lb');
        createParamTime = createParamTime + toc;
    end
    
    % Upper bounds
    temp_ub = ub(idx);
    stages(i).dims.u = sum(temp_ub ~= Inf); % number of upper bounds 
    stages(i).ineq.b.ubidx = find(temp_ub ~= Inf)'; % index vector for upper bounds
    stages(i).ineq.b.ub = full(temp_ub(temp_ub ~= Inf));    % upper bounds
    
    % Create parameter only if there are bounds
    if ~isempty(stages(i).ineq.b.ubidx)
        % Is ub a parameter?
        tic;
        findRelevantQcqpParamsAndCreateForcesParameter( ...
                i,idx(stages(i).ineq.b.ubidx),1,size(ub),qcqpParams.ub,qcqpParamIndices.ub,'ineq.b.ub');
        createParamTime = createParamTime + toc;
    end
    
    % Linear inequalities
    ineq_idx = find(sum(Aineq_temp(:,idx)~=0, 2))';
    stages(i).dims.p = length(ineq_idx); % number of polytopic constraints
    if ~isempty(ineq_idx)
        stages(i).ineq.p.A = full(Aineq(ineq_idx, idx)); % Jacobian of linear inequality 
        stages(i).ineq.p.b = full(bineq(ineq_idx)); % RHS of linear inequality

        % Is p.A a parameter?
        % compute sparsity
        local_Aineq_temp = Aineq_temp(ineq_idx,idx);
        nonzeros = find(local_Aineq_temp);
        if length(nonzeros)/numel(Aineq_temp) <= SPARSITY_THRESHOLD % is Aineq sparse?
            % compute location of nonzeros in big matrix
            [i1, i2] = ind2sub(size(local_Aineq_temp),nonzeros);
            nonzeros_idx = sub2ind(size(Aineq), ineq_idx(i1), idx(i2));
            tic;
            param = findRelevantQcqpParamsAndCreateSparseForcesParameter( ...
                    i,nonzeros_idx,local_Aineq_temp,qcqpParams.Aineq,qcqpParamIndices.Aineq,'ineq.p.A');
            if ~isempty(param)
                standardParamValues.(param.name) = stages(i).ineq.p.A(nonzeros);
                stages(i).ineq.p.A = [];
            end
            createParamTime = createParamTime + toc;
        else % Aineq is dense
            tic;
            findRelevantQcqpParamsAndCreateForcesParameter( ...
                    i,ineq_idx,idx,size(Aineq),qcqpParams.Aineq,qcqpParamIndices.Aineq,'ineq.p.A');
            createParamTime = createParamTime + toc;
        end
        
        % Is p.b a parameter?
        tic;
        findRelevantQcqpParamsAndCreateForcesParameter( ...
                i,ineq_idx,1,size(bineq),qcqpParams.bineq,qcqpParamIndices.bineq,'ineq.p.b');
        createParamTime = createParamTime + toc;
    end
    
    
    % Quadratic constraints
    stages(i).dims.q = 0; % number of quadratic constraints
    if ~isempty(Q)
        stages(i).ineq.q.idx = {}; % index vectors 
        stages(i).ineq.q.Q = {}; % Hessians
        stages(i).ineq.q.l = {}; % linear terms
        stages(i).ineq.q.r = []; % RHSs 

        for k=1:numel(Q) % go through each quad. inequality
            subQ = Q_temp{k}(idx,idx);
            subL = l_temp(idx,k);
            if any(any(subQ)) || any(subL) % Inequality is relevant
                quad_idx = [];
                for s=1:size(subQ,1)
                    quad_idx = [quad_idx find(subQ(s,:)~=0)];
                    if subL(s)~=0
                        quad_idx(end+1) = s;
                    end
                end
                quad_idx = sort(unique(quad_idx));
                
                stages(i).ineq.q.idx{end+1} = quad_idx;
                stages(i).ineq.q.Q{end+1} = full(Q{k}(idx(quad_idx),idx(quad_idx)));
                stages(i).ineq.q.l{end+1} = full(l(idx(quad_idx),k)); % linear terms
                stages(i).ineq.q.r(end+1) = full(r(k)); % RHSs 
                stages(i).dims.q = stages(i).dims.q + 1;
                    
                orig_idx = idx(quad_idx);
                
                % Is Q a param? (Q,l,r are special cases because of k)
                tic;
                param = findRelevantQcqpParamsAndCreateForcesParameter( ...
                    i,orig_idx,orig_idx,size(H),qcqpParams.Q,qcqpParamIndices.Q,'ineq.q.Q', ...
                    k); % maps2mat argument!
                % important: set maps2mat (not set in function) and set
                % standard value
                if ~isempty(param)
                    params(p-1).maps2mat = stages(i).dims.q;
                    standardParamValues.(param.name) = stages(i).ineq.q.Q{end};
                    stages(i).ineq.q.Q{end} = [];
                end
                createParamTime = createParamTime + toc;
                
                % Is l a param?
                tic;
                param = findRelevantQcqpParamsAndCreateForcesParameter( ...
                    i,orig_idx,1,size(f),qcqpParams.l,qcqpParamIndices.l,'ineq.q.l', ...
                    k); % maps2mat argument!
                % important: set maps2mat (not set in function) and set
                % standard value
                if ~isempty(param)
                    params(p-1).maps2mat = stages(i).dims.q;
                    standardParamValues.(param.name) = stages(i).ineq.q.l{end};
                    stages(i).ineq.q.l{end} = [];
                end
                createParamTime = createParamTime + toc;
                
                % Is r a param?
                tic;
                param = findRelevantQcqpParamsAndCreateForcesParameter( ...
                    i,1,k,size(r),qcqpParams.r,qcqpParamIndices.r,'ineq.q.r',...
                    1); % maps2mat argument --> don't set standard value
                
                % important: set maps2mat (not set in function) and set
                % standard value
                if ~isempty(param)
                    params(p-1).maps2mat = stages(i).dims.q;
                    standardParamValues.(param.name) = stages(i).ineq.q.r(end);
                    stages(i).ineq.q.r(end) = 0;
                end
                createParamTime = createParamTime + toc;
            end
        end
    end
    
    % Assign binary variables, if there are any
    bidx = find(ismember(idx,qcqpParams.bidx));
    if ~isempty(bidx)
        stages(i).bidx = bidx;
    end
    
    % Assign boundaries on binary variables if needed
    if isfield(stages(i),'bidx')
        for j=stages(i).bidx
            if ~ismember(j,stages(i).ineq.b.lbidx)
                stages(i).ineq.b.lbidx(end+1) = j;
                stages(i).ineq.b.lb(end+1) = 0;
                stages(i).dims.l = stages(i).dims.l + 1;
            end
            if ~ismember(j,stages(i).ineq.b.ubidx)
                stages(i).ineq.b.ubidx(end+1) = j;
                stages(i).ineq.b.ub(end+1) = 1;
                stages(i).dims.u = stages(i).dims.u + 1;
            end
        end
    end
    
    last_idx = idx;
    
    status = y2f_progressbar(status,i,G.n,barwidth);
end

    function result = computeLinearIndices(matrixSize, row_idx, col_idx)
    % Helper function that returns array of indices for all elements in the
    % specified rows and columns 
    
        if ~isempty(row_idx) && ~isempty(col_idx)
            m = length(row_idx);
            n = length(col_idx);
            cols = repmat(col_idx,m,1);
            result = sub2ind(matrixSize, repmat(row_idx,1,n), cols(:)');
        else
            result = zeros(0,1);
        end
    end

    function param = findRelevantQcqpParamsAndCreateDiagonalForcesParameter( stage, row_idx, fullMatrixSize, qcqpParams, qcqpParamIndices, param_name)
    % Helper function to create diagonal FORCES parameters if necessary
    % Attention: standard value is not set by this function!
    % Do it after calling it with 'standardParamValues.(param.name) = ...'
    
        param = [];
    
        % Find relevant QCQP parameters (=> check indices in big matrix)
        element_idx = sub2ind(fullMatrixSize, row_idx, row_idx);
        [relevant_params,param_local_idx] = ismember(qcqpParamIndices, element_idx); % find positions of parameters
        % relevant_params(j) is 1 iff QCQP param at index j is needed
        % param_local_idx(j) contains the (local) index of the element
        % influenced by this parameter
        
        if any(relevant_params) % We have QCQP params --> create a FORCES param            
            % Create FORCES param with standard value
            param_id = sprintf('p_%u',p);
            params(p) = newParam(param_id, stage, param_name, 'diag');
            
            % Make param map (additive parts of parameter, see help of
            % generateStagesFromGraph)
            forcesParamMap.(param_id) = zeros(4,0);
            for j=find(relevant_params)
                forcesParamMap.(param_id)(:,end+1) = [param_local_idx(j);...
                                                      qcqpParams(j).factor;...
                                                      yalmipParamMap(:,qcqpParams(j).maps2origparam)];
            end
            
            param = params(p);
            p = p+1;
        end
    
    end

    function param = findRelevantQcqpParamsAndCreateSparseForcesParameter( stage, nonzero_idx, sparsityPattern, qcqpParams, qcqpParamIndices, param_name)
    % Helper function to create sparse FORCES parameters if necessary
    % Attention: standard value is not set by this function!
    % Do it after calling it with 'standardParamValues.(param.name) = ...'
    
        param = [];
    
        % Find relevant QCQP parameters (=> check indices in big matrix)
        [relevant_params,param_local_idx] = ismember(qcqpParamIndices, nonzero_idx); % find positions of parameters
        % relevant_params(j) is 1 iff QCQP param at index j is needed
        % param_local_idx(j) contains the (local) index of the element
        % influenced by this parameter
        
        if any(relevant_params) % We have QCQP params --> create a FORCES param
            % Create FORCES param
            param_id = sprintf('p_%u',p);
            params(p) = newParam(param_id, stage, param_name, 'sparse', full(sparsityPattern));
            
            % Make param map (additive parts of parameter, see help of
            % generateStagesFromGraph)
            forcesParamMap.(param_id) = zeros(4,0);
            for j=find(relevant_params)
                forcesParamMap.(param_id)(:,end+1) = [param_local_idx(j);...
                                                      qcqpParams(j).factor;...
                                                      yalmipParamMap(:,qcqpParams(j).maps2origparam)];
            end

            param = params(p);
            p = p+1;
        end
    
    end

    function param = findRelevantQcqpParamsAndCreateForcesParameter( stage, row_idx, col_idx, fullMatrixSize, qcqpParams, qcqpParamIndices, param_name, maps2mat)
    % Helper function to create FORCES parameters if necessary
    
        param = [];
    
        % Find relevant QCQP parameters (=> check indices in big matrix)
        element_idx = computeLinearIndices(fullMatrixSize, row_idx, col_idx);
        [relevant_params,param_local_idx] = ismember(qcqpParamIndices, element_idx); % find positions of parameters
        % relevant_params(j) is 1 iff QCQP param at index j is needed
        % param_local_idx(j) contains the (local) index of the element
        % influenced by this parameter
        
        % special case: maps2mat was set (we're dealing with Q or l)
        if nargin >= 8
            % filter params that do not affect this "mat"
            not_affected_params = [qcqpParams.maps2mat] ~= maps2mat;
            relevant_params(not_affected_params) = 0;
            param_local_idx(not_affected_params) = 0;
        end
        
        if any(relevant_params) % We have QCQP params --> create a FORCES param            
            % Create FORCES param with standard value
            param_id = sprintf('p_%u',p);
            params(p) = newParam(param_id, stage, param_name);
            if nargin <= 7 % no maps2mat (we can't set standardParamValues for that case)
                var = regexp(param_name,'[.]','split');  % split name
                if length(var) == 2
                    standardParamValues.(param_id) = stages(stage).(genvarname(var{1})).(genvarname(var{2}));
                    stages(stage).(genvarname(var{1})).(genvarname(var{2})) = [];
                elseif length(var) == 3
                    standardParamValues.(param_id) = stages(stage).(genvarname(var{1})).(genvarname(var{2})).(genvarname(var{3}));
                    stages(stage).(genvarname(var{1})).(genvarname(var{2})).(genvarname(var{3})) = [];
                else
                    error('This case exists?');
                end
            end
            
            % Make param map (additive parts of parameter, see help of
            % generateStagesFromGraph)
            forcesParamMap.(param_id) = zeros(4,0);
            for j=find(relevant_params)
                forcesParamMap.(param_id)(:,end+1) = [param_local_idx(j);...
                                                      qcqpParams(j).factor;...
                                                      yalmipParamMap(:,qcqpParams(j).maps2origparam)];
            end
            
            param = params(p);
            p = p+1;
        end
    
    end

%     function createDiagonalCostParameter(stage, relevantParams, element_idx)
%     % Helper function to create a FORCES parameters for a diagonal cost
%         params(p) = newParam(sprintf('p_%u',p), stage, 'cost.H', 'diag');
%         standardParamValues.(sprintf('p_%u',p)) = stages(stage).cost.H(logical(eye(length(element_idx)))); % only use diagonal
%         stages(stage).cost.H = [];
% 
%         forcesParamMap.(sprintf('p_%u',p)) = zeros(4,0);
%         paramSize = size(standardParamValues.(sprintf('p_%u',p)));
%         for j=relevantParams
%             for element=1:length(element_idx)
%                 value_idx = find(sub2ind(size(H),element_idx(element),element_idx(element)) == ...
%                         qcqpParams.H(j).maps2index);
%                 if length(value_idx) == 1
%                     forcesParamMap.(sprintf('p_%u',p))(:,end+1) = [element;...
%                                                                    qcqpParams.H(j).factor;...
%                                                                    yalmipParamMap(:,qcqpParams.H(j).maps2origparam)];
%                 elseif length(value_idx) > 1
%                     error('Mistake in the new stages formulation')
%                 end
%             end
%         end
%         p = p + 1;
%     end
            
fprintf('\nTime spent in createParameter: %5.1f seconds\n', createParamTime);

end




function [components,stages,params,standardParamValues,forcesParamMap] = stagesFromPathGraphs(components,H,f,Aineq,bineq,Aeq,beq,l,Q,r,lb,ub,qcqpParams,yalmipParamMap,outputIndices)
%STAGESFROMPATHGRAPHS Creates stages for a graph that has been split into 
%path graphs. This function just combines the stages for each subgraph into
%a cell array. The real conversion is done by GENERATESTAGESFROMGRAPH.
%   Input:
%       components                  cell array of connected graphs
%       H,f,Aineq,bineq,Aeq,beq,    standard QCQP matrices
%           l,Q,r,lb,ub
%       qcqpParams                  see STAGES2PARAMQCQP
%       yalmipParamMap              parameter map for YALMIP parameters.
%                                   (matrix)
%                                   Each column corresponds to the
%                                   parameter with the same index.
%                                   1st row: index of matrix with values, 
%                                   2nd row: index of element inside matrix
%       outputIndices               variable indices of variables relevant
%                                   for output (row vector)
%       
%   Output:       
%       components              modified components list, subgraphs not 
%                               relevant for ouput are removed
%       stages                  cell array of stages (see
%                               GENERATESTAGESFROMGRAPH) 
%       params                  cell array of params (see
%                               GENERATESTAGESFROMGRAPH) 
%       standardParamValues     cell array of standardParamValues (see
%                               GENERATESTAGESFROMGRAPH) 
%       forcesParamMap          cell array of forcesParamMaps (see
%                               GENERATESTAGESFROMGRAPH) 
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

stages = {};
params = {};
standardParamValues = {};
forcesParamMap = {};

% Go through all components and convert them to stages
removeIdx = []; % don't forget to delete these
for i=1:numel(components)
    if any(ismember(outputIndices, cell2mat(components{i}.vertices)))
        % Construct stages using path graph
        [stages_temp,params_temp,standardParamValues_temp,forcesParamMap_temp] = generateStagesFromGraph(components{i},H,f,Aineq,bineq,Aeq,beq,l,Q,r,lb,ub,qcqpParams,yalmipParamMap);
        
        % Hook new stages together with other stages
        stages{end+1} = stages_temp;
        params{end+1} = params_temp;
        standardParamValues{end+1} = standardParamValues_temp;
        forcesParamMap{end+1} = forcesParamMap_temp;
    else % we can delete component
        removeIdx(end+1) = i;
    end
end

components(removeIdx) = [];

end
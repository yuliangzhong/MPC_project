function [ success ] = generateSimulinkBlock( self )
%GENERATESIMULINKBLOCK Create a model containing a Simulink block for the
%solver
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

success = 0;

% Get solver name from option
solverName = self.default_codeoptions.name;

% Check if FORCES solver has been generated
if (exist(solverName,'dir') == 0)
    error('Solver ''%s'' has not been generated!', solverName)
end

library = 'y2f_simulink_lib';
block = solverName;

% Create Simulink library
if exist(library) % library found, no need to create one
    disp('Found existing Y2F Simulink library.')
    load_system(library);
    
    % we need to unlock the library to edit it
    set_param(library,'Lock','off') 
else % no library found
    disp('Y2F Simulink library not found, creating new one.')
    
    % Check if slblocks.m exists, if yes ask user if we can overwrite it
    if ~isempty(dir('slblocks.m'))
        while 1
            beep
            in = input(['[' 8 'slblocks.m already exists in the current folder. Y2F needs to overwrite it to create the Simulink library.\nDo you want to continue [y/n]? ]' 8],'s');
            if strcmpi(in,'y')
                break
            elseif strcmpi(in,'n')
                error('Y2F could not create Simulink library.');
            end
        end
    end
    
    % Create library
    new_system(library, 'Library');
    
    % Create slblocks.m to "register" Y2F library
    mFileID = fopen('slblocks.m','w');
    
    fprintf(mFileID, 'function blkStruct = slblocks\n');
    fprintf(mFileID, '%% This function specifies that the Y2F library should appear in the\n');
    fprintf(mFileID, '%% Library Browser and be cached in the browser repository\n');
    fprintf(mFileID, 'Browser.Library = ''y2f_simulink_lib'';\n');
    fprintf(mFileID, 'Browser.Name = ''Y2F FORCES PRO Solvers'';\n');
    fprintf(mFileID, 'blkStruct.Browser = Browser;\n');
    
    fclose(mFileID);

end

% Add S-function block if it doesn't exist already
blocks = find_system(library);
if ~any(cell2mat(strfind(blocks,[library '/' block])))
    add_block('built-in/S-Function', [library '/' block]);
    
    % Create mask
    mask = Simulink.Mask.create([library '/' block]);
else
    % We need to load the mask to edit it
    mask = Simulink.Mask.get([library '/' block]);
end

% Set right parameters, ports, etc. for block
set_param([library '/' block], 'FunctionName', [solverName '_simulinkBlock']);

% Create mask to name input/output ports and add image
iconDrawingString = 'image(''forcesprologo.jpg'', ''center'', ''on'')';
for i=1:self.numParams
    iconDrawingString = sprintf('%s;port_label(''input'', %u, ''%s'')', iconDrawingString, i, self.paramNames{i});
end
for i=1:numel(self.outputSize)
    iconDrawingString = sprintf('%s;port_label(''output'', %u, ''%s'')', iconDrawingString, i, self.outputNames{i});
end
if (isfield(self.default_codeoptions,'showinfo') && self.default_codeoptions.showinfo) % we have diagnostic fields
    iconDrawingString = sprintf('%s;port_label(''output'', %u, ''exitflag'')',iconDrawingString,numel(self.outputSize)+1);
    iconDrawingString = sprintf('%s;port_label(''output'', %u, ''iterations'')',iconDrawingString,numel(self.outputSize)+2);
    iconDrawingString = sprintf('%s;port_label(''output'', %u, ''solve_time'')',iconDrawingString,numel(self.outputSize)+3);
    iconDrawingString = sprintf('%s;port_label(''output'', %u, ''primal_obj'')',iconDrawingString,numel(self.outputSize)+4);
end
set_param([library '/' block], 'MaskDisplay', iconDrawingString)

% Set position of block
set_param([library '/' block], 'Position', [170, 99, 550, 200])

% Generate description and help
desc = sprintf(['---- Simulink block encapsulating your customized solver %s ----\n\n' ...
    '%s : A fast customized optimization solver.'], solverName, solverName);
set_param([library '/' block], 'MaskDescription', desc);

help = sprintf('%s_simulinkBlock provides an easy Simulink interface for simulating your customized solver.\n\n',solverName);
help = sprintf('%sOUTPUTS = %s(INPUTS) solves an optimization problem where:\n\n', help, solverName);
help = sprintf('%sINPUTS:\n', help);

for i=1:self.numParams
    if self.paramSizes(i,1) == 1 && self.paramSizes(i,2) == 1 % scalar parameter
        help = sprintf('%s- %s (a scalar)\n',help,self.paramNames{i});
    elseif self.paramSizes(i,1) == 1 % row vector
        help = sprintf('%s- %s (row vector of length %u)\n',help,self.paramNames{i},self.paramSizes(i,2));
    elseif self.paramSizes(i,2) == 1 % column vector
        help = sprintf('%s- %s (column vector of length %u)\n',help,self.paramNames{i},self.paramSizes(i,1));
    else
        help = sprintf('%s- %s (matrix of size [%u x %u])\n',help,self.paramNames{i},self.paramSizes(i,1),self.paramSizes(i,2));
    end
end

help = sprintf('%s\nOUTPUTS:\n',help);

for i=1:numel(self.outputSize)
    if self.outputSize{i}(1) == 1 && self.outputSize{i}(2) == 1 % scalar parameter
        help = sprintf('%s- %s (a scalar)\n',help,self.outputNames{i});
    elseif self.outputSize{i}(1) == 1 % row vector
        help = sprintf('%s- %s (row vector of length %u)\n',help,self.outputNames{i},self.outputSize{i}(2));
    elseif self.outputSize{i}(2) == 1 % column vector
        help = sprintf('%s- %s (column vector of length %u)\n',help,self.outputNames{i},self.outputSize{i}(1));
    else
        help = sprintf('%s- %s (matrix of size [%u x %u])\n',help,self.outputNames{i},self.outputSize{i}(1),self.outputSize{i}(2));
    end
end

help = sprintf('%s\nFor more information, see https://www.embotech.com/FORCES-Pro/How-to-use/Simulink-Interface/Simulink-Block',help);

set_param([library '/' block], 'MaskHelp', help);

% Save system
%set_param(gcs,'EnableLBRepository','on'); % enable library in browser
set_param(library,'EnableLBRepository','on'); % enable library in browser
filename = save_system(library);
close_system(library, 0);

success = 1;

end

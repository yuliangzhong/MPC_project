function [ success ] = generateMEXInterfaceCode( self )
%GENERATEMEXINTERFACECODE generates MEX C code that will prepare the
%parameters for the FORCES solver. It also assembles the correct outputs.
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

success = 0;

% Get solver name from option
solverName = self.default_codeoptions.name;
if(isfield(self.default_codeoptions, 'certification') && self.default_codeoptions.certification == 1)
    solverName_constant = upper(solverName);
else
    solverName_constant = solverName;
end

% Check if FORCES solver has been generated
if (exist(solverName,'dir') == 0)
    error('Solver ''%s'' has not been generated!', solverName)
end

fileID = fopen([solverName,filesep,'interface',filesep,solverName,'_mex.c'],'w');

% Write standard comment
fprintf(fileID, '/*\n This is an interface for %s that ',solverName);
fprintf(fileID, 'is used by optimizerFORCES to call the solver\n');
fprintf(fileID, '*/ \n\n');

% Includes
fprintf(fileID, '#include "mex.h"\n');
fprintf(fileID, '#include "math.h"\n');
fprintf(fileID, '#include "../include/%s.h"\n',solverName,solverName);
fprintf(fileID, '#include <stdio.h>\n\n');

% Copy functions stolen from FORCES
fprintf(fileID, '/* copy functions */\n');
fprintf(fileID, 'void copyCArrayToM(double *src, double *dest, int dim) {\n');
fprintf(fileID, '\twhile (dim--) {\n');
fprintf(fileID, '\t\t*dest++ = (double)*src++;\n');
fprintf(fileID, '\t}\n');
fprintf(fileID, '}\n');
fprintf(fileID, 'void copyMArrayToC(double *src, double *dest, int dim) {\n');
fprintf(fileID, '\twhile (dim--) {\n');
fprintf(fileID, '\t\t*dest++ = (double) (*src++) ;\n');
fprintf(fileID, '\t}\n');
fprintf(fileID, '}\n\n');

% Arguments for solver(s)
fprintf(fileID, '/* Some memory for mex-function */\n');
fprintf(fileID, 'static %s_params params;\n',solverName);
fprintf(fileID, 'static %s_output output;\n',solverName);
fprintf(fileID, 'static %s_info info;\n\n',solverName);

% Start of MEX function
fprintf(fileID, '/* THE mex-function */\n');
fprintf(fileID, 'void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {\n');
fprintf(fileID, '\t/* file pointer for printing */\n');
fprintf(fileID, '\tFILE *fp = NULL;\n\n');
    
fprintf(fileID, '\t/* define variables */\n');
fprintf(fileID, '\tmxArray *par;\n\n');
fprintf(fileID, '\tconst mxArray *param_values = prhs[0];\n');
fprintf(fileID, '\tmxArray *outvar;\n');
fprintf(fileID, '\tint i;\n');
if self.numSolvers == 1
    fprintf(fileID, '\tint exitflag;\n');
else
    fprintf(fileID, '\tint *exitflag;\n');
end
fprintf(fileID, '\tconst char *fname;\n\n');

% Solver info fields
if ~isfield(self.default_codeoptions, 'solvemethod') || ~strcmpi(self.default_codeoptions.solvemethod, 'ADMM') % not for ADMM
    fprintf(fileID, '\tconst char *infofields[16] = { "it", "it2opt", "res_eq", "res_ineq",  "pobj",  "dobj",  "dgap", "rdgap",  "mu",  "mu_aff",  "sigma",  "lsit_aff",  "lsit_cc",  "step_aff",   "step_cc",  "solvetime"};\n\n');
else % ADMM has slightly different fields
    fprintf(fileID, '\tconst char *infofields[9] = { "it", "it2opt", "res_eq", "res_dual",  "pobj",  "dobj",  "dgap", "rdgap", "solvetime"};\n\n');    
end

% Check number of in- and outputs
fprintf(fileID, '\t/* Check for proper number of arguments */\n');
fprintf(fileID, '\tif (nrhs != 1) {\n');
fprintf(fileID, '\t\tmexErrMsgTxt("This function requires exactly 1 input: parameter value cell array.");\n');
fprintf(fileID, '\t}\n');
fprintf(fileID, '\tif (nlhs > 3) {\n');
fprintf(fileID, '\t\tmexErrMsgTxt("This function returns at most 3 outputs.");\n');
fprintf(fileID, '\t}\n');

% Check type of input
fprintf(fileID, '\t/* Check whether params is actually a cell array */\n');
fprintf(fileID, '\tif( !mxIsCell(param_values) ) {\n');
fprintf(fileID, '\t\tmexErrMsgTxt("%s requires a cell array as input.");\n',solverName);
fprintf(fileID, '\t}\n\n');

% Check length of input
fprintf(fileID, '\t/* Check whether params has the right number of elements */\n');
fprintf(fileID, '\tif( mxGetNumberOfElements(param_values) != %u ) {\n',self.numParams);
fprintf(fileID, '\t\tmexErrMsgTxt("Input must have %u elements.");\n',self.numParams);
fprintf(fileID, '\t}\n\n');

% Check sizes of parameter values and load their values
fprintf(fileID, '\t/* Load param values into C arrays and check their size */\n');
for i=1:self.numParams
    % Get cell element
    fprintf(fileID, '\tpar = mxGetCell(param_values,%u);\n',i-1);
    
    % Cell element not found
    fprintf(fileID, '\tif( par == NULL ) {\n');
    fprintf(fileID, '\t\tmexErrMsgTxt("Parameter #%u not found");\n',i);
    fprintf(fileID, '\t}\n');
    
    % Parameter value is not double
    fprintf(fileID, '\tif( !mxIsDouble(par) ) {\n');
    fprintf(fileID, '\t\tmexErrMsgTxt("Parameter #%u must be a double.");\n',i);
    fprintf(fileID, '\t}\n');
    
    % Check parameter value size
    fprintf(fileID, '\tif( mxGetM(par) != %u || mxGetN(par) != %u ) {\n',self.paramSizes(i,1),self.paramSizes(i,2));
    fprintf(fileID, '\t\tmexErrMsgTxt("Parameter #%u must be of size [%u x %u]");\n',i,self.paramSizes(i,1),self.paramSizes(i,2));
    fprintf(fileID, '\t}\n');
    
    % Copy parameter value
    fprintf(fileID, '\tcopyMArrayToC(mxGetPr(par),params.%s,%u);\n\n',self.paramNames{i},prod(self.paramSizes(i,:)));
end
    
% If the user wanted output, we need to store it
fprintf(fileID, '\t#if SET_PRINTLEVEL_%s > 0\n',solverName_constant);
fprintf(fileID, '\t\t/* Prepare file for printfs */\n');
fprintf(fileID, '\t\t/*fp = freopen("stdout_temp","w+",stdout);*/\n');
fprintf(fileID, '\t\tfp = fopen("stdout_temp","w+");\n');
fprintf(fileID, '\t\tif( fp == NULL ) {\n');
fprintf(fileID, '\t\t\tmexErrMsgTxt("freopen of stdout did not work.");\n');
fprintf(fileID, '\t\t}\n');
fprintf(fileID, '\t\trewind(fp);\n');
fprintf(fileID, '\t#endif\n\n');

fprintf(fileID, '\t/* call solver */\n');
fprintf(fileID, '\texitflag = %s_solve(&params, &output, &info, fp );\n\n',solverName);

% Print output in console
fprintf(fileID, '\t#if SET_PRINTLEVEL_%s > 0\n',solverName_constant);
fprintf(fileID, '\t\t/* Read contents of printfs printed to file */\n');
fprintf(fileID, '\t\trewind(fp);\n');
fprintf(fileID, '\t\twhile( (i = fgetc(fp)) != EOF ) {\n');
fprintf(fileID, '\t\t\tmexPrintf("%%c",i);\n');
fprintf(fileID, '\t\t}\n');
fprintf(fileID, '\t\tfclose(fp);\n');
fprintf(fileID, '\t#endif\n\n');
    
% Put outputs together
if self.outputIsCell % multiple outputs possible
    % Create MATLAB output cell array
    fprintf(fileID, '\tplhs[0] = mxCreateCellMatrix(1, %u);\n',numel(self.outputBase));
    for i=1:numel(self.outputBase) % every output has a base
        outSize = self.outputSize{i};
        fprintf(fileID, '\toutvar = mxCreateDoubleMatrix(%u, %u, mxREAL);\n',outSize(1),outSize(2));
        fprintf(fileID, '\tcopyCArrayToM(output.%s, mxGetPr(outvar), %u);\n',self.outputNames{i},prod(outSize));
        fprintf(fileID, '\tmxSetCell(plhs[0], %u, outvar);\n\n',i-1);
    end
else % only one output
    % Create MATLAB output matrix
    outSize = self.outputSize{1};
    fprintf(fileID, '\tplhs[0] = mxCreateDoubleMatrix(%u, %u, mxREAL);\n',outSize(1),outSize(2));
    fprintf(fileID, '\tcopyCArrayToM(output.%s, mxGetPr(plhs[0]), %u);\n\n',self.outputNames{1},prod(outSize));
end

% add other output arguments (exitflags and info)
if self.numSolvers > 1
    fprintf(fileID, '\t/* copy exitflags */\n');
    fprintf(fileID, '\tif( nlhs > 1 ) {\n');
    fprintf(fileID, '\t\tplhs[1] = mxCreateDoubleMatrix(1, %u, mxREAL);\n',self.numSolvers);
    for i=1:self.numSolvers
        fprintf(fileID, '\t\t*(mxGetPr(plhs[1])+%u) = (double)exitflag[%u];\n',i-1,i-1);
    end
    fprintf(fileID, '\t}\n\n');

    fprintf(fileID, '\t/* copy info structs */\n');
    fprintf(fileID, '\tif( nlhs > 2 ) {\n');
    if isfield(self.default_codeoptions, 'solvemethod') && strcmpi(self.default_codeoptions.solvemethod, 'ADMM') % ADMM has different fields
        fprintf(fileID, '\t\tplhs[2] = mxCreateStructMatrix(%u, 1, 9, infofields);\n\n', self.numSolvers);
    else
        fprintf(fileID, '\t\tplhs[2] = mxCreateStructMatrix(%u, 1, 16, infofields);\n\n', self.numSolvers);
    end

    for i=1:self.numSolvers
        fprintf(fileID, '\t\t/* info for solver #%u */\n',i);

        fprintf(fileID, '\t\t/* iterations */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = (double)info.it[%u];\n',i-1);
        fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "it", outvar);\n\n',i-1);

        fprintf(fileID, '\t\t/* iterations to optimality (branch and bound) */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = (double)info.it2opt[%u];\n',i-1);
        fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "it2opt", outvar);\n\n',i-1);

        fprintf(fileID, '\t\t/* res_eq */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.res_eq[%u];\n',i-1);
        fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "res_eq", outvar);\n\n',i-1);

        if isfield(self.default_codeoptions, 'solvemethod') && strcmpi(self.default_codeoptions.solvemethod, 'ADMM') % extra field for ADMM
            fprintf(fileID, '\t\t/* res_dual */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = info.res_dual[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "res_dual", outvar);\n\n',i-1);
        else
            fprintf(fileID, '\t\t/* res_ineq */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = info.res_ineq[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "res_ineq", outvar);\n\n',i-1);            
        end

        fprintf(fileID, '\t\t/* pobj */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.pobj[%u];\n',i-1);
        fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "pobj", outvar);\n\n',i-1);

        fprintf(fileID, '\t\t/* dobj */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.dobj[%u];\n',i-1);
        fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "dobj", outvar);\n\n',i-1);

        fprintf(fileID, '\t\t/* dgap */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.dgap[%u];\n',i-1);
        fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "dgap", outvar);\n\n',i-1);

        fprintf(fileID, '\t\t/* rdgap */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.rdgap[%u];\n',i-1);
        fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "rdgap", outvar);\n\n',i-1);

        if ~isfield(self.default_codeoptions, 'solvemethod') || ~strcmpi(self.default_codeoptions.solvemethod, 'ADMM') % not for ADMM
            fprintf(fileID, '\t\t/* mu */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = info.mu[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "mu", outvar);\n\n',i-1);

            fprintf(fileID, '\t\t/* mu_aff */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = info.mu_aff[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "mu_aff", outvar);\n\n',i-1);

            fprintf(fileID, '\t\t/* sigma */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = info.sigma[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "sigma", outvar);\n\n',i-1);

            fprintf(fileID, '\t\t/* lsit_aff */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = (double)info.lsit_aff[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "lsit_aff", outvar);\n\n',i-1);

            fprintf(fileID, '\t\t/* lsit_cc */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = (double)info.lsit_cc[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "lsit_cc", outvar);\n\n',i-1);

            fprintf(fileID, '\t\t/* step_aff */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = info.step_aff[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "step_aff", outvar);\n\n',i-1);

            fprintf(fileID, '\t\t/* step_cc */\n');
            fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
            fprintf(fileID, '\t\t*mxGetPr(outvar) = info.step_cc[%u];\n',i-1);
            fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "step_cc", outvar);\n\n',i-1);
        end

        fprintf(fileID, '\t\t/* solver time */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.solvetime[%u];\n',i-1);
        fprintf(fileID, '\t\tmxSetField(plhs[2], %u, "solvetime", outvar);\n\n',i-1);
    end
    fprintf(fileID, '\t}\n');
else % only one solver --> no cell arrays necessary
    fprintf(fileID, '\t/* copy exitflag */\n');
    fprintf(fileID, '\tif( nlhs > 1 ) {\n');
    fprintf(fileID, '\t\tplhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*(mxGetPr(plhs[1])) = (double)exitflag;\n');
    fprintf(fileID, '\t}\n\n');

    fprintf(fileID, '\t/* copy info struct */\n');
    fprintf(fileID, '\tif( nlhs > 2 ) {\n');
    if isfield(self.default_codeoptions, 'solvemethod') && strcmpi(self.default_codeoptions.solvemethod, 'ADMM') % ADMM has different fields
        fprintf(fileID, '\t\tplhs[2] = mxCreateStructMatrix(1, 1, 9, infofields);\n\n');
    else
        fprintf(fileID, '\t\tplhs[2] = mxCreateStructMatrix(1, 1, 16, infofields);\n\n');
    end

    fprintf(fileID, '\t\t/* iterations */\n');
    fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*mxGetPr(outvar) = (double)info.it;\n');
    fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "it", outvar);\n\n');

    fprintf(fileID, '\t\t/* iterations to optimality (branch and bound) */\n');
    fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*mxGetPr(outvar) = (double)info.it2opt;\n');
    fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "it2opt", outvar);\n\n');

    fprintf(fileID, '\t\t/* res_eq */\n');
    fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*mxGetPr(outvar) = info.res_eq;\n');
    fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "res_eq", outvar);\n\n');

    if isfield(self.default_codeoptions, 'solvemethod') && strcmpi(self.default_codeoptions.solvemethod, 'ADMM') % extra field for ADMM
        fprintf(fileID, '\t\t/* res_dual */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.res_dual;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "res_dual", outvar);\n\n');
    else % not for ADMM
        fprintf(fileID, '\t\t/* res_ineq */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.res_ineq;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "res_ineq", outvar);\n\n');
    end

    fprintf(fileID, '\t\t/* pobj */\n');
    fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*mxGetPr(outvar) = info.pobj;\n');
    fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "pobj", outvar);\n\n');

    fprintf(fileID, '\t\t/* dobj */\n');
    fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*mxGetPr(outvar) = info.dobj;\n');
    fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "dobj", outvar);\n\n');

    fprintf(fileID, '\t\t/* dgap */\n');
    fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*mxGetPr(outvar) = info.dgap;\n');
    fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "dgap", outvar);\n\n');

    fprintf(fileID, '\t\t/* rdgap */\n');
    fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*mxGetPr(outvar) = info.rdgap;\n');
    fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "rdgap", outvar);\n\n');

    if ~isfield(self.default_codeoptions, 'solvemethod') || ~strcmpi(self.default_codeoptions.solvemethod, 'ADMM') % not for ADMM
        fprintf(fileID, '\t\t/* mu */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.mu;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "mu", outvar);\n\n');

        fprintf(fileID, '\t\t/* mu_aff */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.mu_aff;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "mu_aff", outvar);\n\n');

        fprintf(fileID, '\t\t/* sigma */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.sigma;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "sigma", outvar);\n\n');

        fprintf(fileID, '\t\t/* lsit_aff */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = (double)info.lsit_aff;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "lsit_aff", outvar);\n\n');

        fprintf(fileID, '\t\t/* lsit_cc */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = (double)info.lsit_cc;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "lsit_cc", outvar);\n\n');

        fprintf(fileID, '\t\t/* step_aff */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.step_aff;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "step_aff", outvar);\n\n');

        fprintf(fileID, '\t\t/* step_cc */\n');
        fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
        fprintf(fileID, '\t\t*mxGetPr(outvar) = info.step_cc;\n');
        fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "step_cc", outvar);\n\n');
    end

    fprintf(fileID, '\t\t/* solver time */\n');
    fprintf(fileID, '\t\toutvar = mxCreateDoubleMatrix(1, 1, mxREAL);\n');
    fprintf(fileID, '\t\t*mxGetPr(outvar) = info.solvetime;\n');
    fprintf(fileID, '\t\tmxSetField(plhs[2], 0, "solvetime", outvar);\n');
    
    fprintf(fileID, '\t}\n');
end

fprintf(fileID, '}'); % end of mex-function
    
% Don't forget to close MEX file
fclose(fileID);

success = 1;

end

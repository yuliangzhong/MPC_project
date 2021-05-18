function [ success ] = generateSimulinkInterfaceCode( self )
%GENERATESIMULINKINTERFACECODE generates a C file with an S-function for
%use in a Simulink model.
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

fileID = fopen([solverName,filesep,'interface',filesep,solverName,'_simulinkBlock.c'],'w');

fprintf(fileID, '/*\n');
fprintf(fileID, '%s : A fast customized optimization solver.\n',solverName);
fprintf(fileID, '*/\n');
fprintf(fileID, '\n');
fprintf(fileID, '#define S_FUNCTION_LEVEL 2\n');
fprintf(fileID, '#define S_FUNCTION_NAME %s_simulinkBlock\n',solverName);
fprintf(fileID, '\n');
fprintf(fileID, '#include "simstruc.h"\n');
fprintf(fileID, '\n');
fprintf(fileID, '/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */\n');
fprintf(fileID, '\n');
fprintf(fileID, '/* include FORCES functions and defs */\n');
fprintf(fileID, '#include "../include/%s.h"\n',solverName);
fprintf(fileID, '\n');
fprintf(fileID, '#if defined(MATLAB_MEX_FILE)\n');
fprintf(fileID, '#include "tmwtypes.h"\n');
fprintf(fileID, '#include "simstruc_types.h"\n');
fprintf(fileID, '#else\n');
fprintf(fileID, '#include "rtwtypes.h"\n');
fprintf(fileID, '#endif\n');
fprintf(fileID, '\n');
fprintf(fileID, '\n');
fprintf(fileID, '/*====================*\n');
fprintf(fileID, ' * S-function methods *\n');
fprintf(fileID, ' *====================*/\n');
fprintf(fileID, '/* Function: mdlInitializeSizes =========================================\n');
fprintf(fileID, ' * Abstract:\n');
fprintf(fileID, ' *   Setup sizes of the various vectors.\n');
fprintf(fileID, ' */\n');
fprintf(fileID, 'static void mdlInitializeSizes(SimStruct *S)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '\n');
fprintf(fileID, '    DECL_AND_INIT_DIMSINFO(inputDimsInfo);\n');
fprintf(fileID, '    DECL_AND_INIT_DIMSINFO(outputDimsInfo);\n');
fprintf(fileID, '    ssSetNumSFcnParams(S, 0);\n');
fprintf(fileID, '     if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {\n');
fprintf(fileID, '	 return; /* Parameter mismatch will be reported by Simulink */\n');
fprintf(fileID, '     }\n');
fprintf(fileID, '\n');
fprintf(fileID, '	/* initialize size of continuous and discrete states to zero */\n');
fprintf(fileID, '    ssSetNumContStates(S, 0);\n');
fprintf(fileID, '    ssSetNumDiscStates(S, 0);\n');
fprintf(fileID, '\n');

% Input ports
fprintf(fileID, '	/* initialize input ports - there are %u in total */\n',self.numParams);
fprintf(fileID, '    if (!ssSetNumInputPorts(S, %u)) return;\n',self.numParams);
fprintf(fileID, '    	\n');

for i=1:self.numParams
    fprintf(fileID, '	/* Input Port %u (%s) */\n',i-1,self.paramNames{i});
    fprintf(fileID, '    ssSetInputPortMatrixDimensions(S, %u, %u, %u);\n', i-1, self.paramSizes(i,1), self.paramSizes(i,2));
    fprintf(fileID, '    ssSetInputPortDataType(S, %u, SS_DOUBLE);\n',i-1);
    fprintf(fileID, '    ssSetInputPortComplexSignal(S, %u, COMPLEX_NO); /* no complex signals suppported */\n',i-1);
    fprintf(fileID, '    ssSetInputPortDirectFeedThrough(S, %u, 1); /* Feedthrough enabled */\n',i-1);
    fprintf(fileID, '    ssSetInputPortRequiredContiguous(S, %u, 1); /*direct input signal access*/ \n',i-1);
    fprintf(fileID, '\n');
end
fprintf(fileID, '\n');

% Output ports
fprintf(fileID, '	/* initialize output ports - there are %u in total */\n',numel(self.outputSize));
if (isfield(self.default_codeoptions,'showinfo') && self.default_codeoptions.showinfo) % Info fields (exitflag, iterations, solve time, pobj) are optional
    fprintf(fileID, '    if (!ssSetNumOutputPorts(S, %u)) return;    \n',numel(self.outputSize) + 4);
else % no info fields
    fprintf(fileID, '    if (!ssSetNumOutputPorts(S, %u)) return;    \n',numel(self.outputSize));
end
fprintf(fileID, '		\n');

for i=1:numel(self.outputSize) % normal output ports
    fprintf(fileID, '	/* Output Port %u */\n',i-1);
    fprintf(fileID, '    ssSetOutputPortMatrixDimensions(S, %u, %u, %u);\n',i-1,self.outputSize{i}(1),self.outputSize{i}(2));
    fprintf(fileID, '    ssSetOutputPortDataType(S, %u, SS_DOUBLE);\n',i-1);
    fprintf(fileID, '    ssSetOutputPortComplexSignal(S, %u, COMPLEX_NO); /* no complex signals suppported */\n',i-1);
    fprintf(fileID, '\n');
end

% Diagnostic info ports
if (isfield(self.default_codeoptions,'showinfo') && self.default_codeoptions.showinfo)
    fprintf(fileID, '	/* Diagnostic Output Ports */\n');
    fprintf(fileID, '    ssSetOutputPortMatrixDimensions(S, %u, 1, 1);\n',numel(self.outputSize)+(0:3));
    fprintf(fileID, '    ssSetOutputPortDataType(S, %u, SS_DOUBLE);\n',numel(self.outputSize)+(0:3));
    fprintf(fileID, '    ssSetOutputPortComplexSignal(S, %u, COMPLEX_NO);\n',numel(self.outputSize)+(0:3));
end

fprintf(fileID, '\n');
fprintf(fileID, '	/* set sampling time */\n');
fprintf(fileID, '    ssSetNumSampleTimes(S, 1);\n');
fprintf(fileID, '\n');
fprintf(fileID, '	/* set internal memory of block */\n');
fprintf(fileID, '    ssSetNumRWork(S, 0);\n');
fprintf(fileID, '    ssSetNumIWork(S, 0);\n');
fprintf(fileID, '    ssSetNumPWork(S, 0);\n');
fprintf(fileID, '    ssSetNumModes(S, 0);\n');
fprintf(fileID, '    ssSetNumNonsampledZCs(S, 0);\n');
fprintf(fileID, '\n');
fprintf(fileID, '    /* Take care when specifying exception free code - see sfuntmpl_doc.c */\n');
fprintf(fileID, '	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ \n');
fprintf(fileID, '	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ \n');
fprintf(fileID, '    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |\n');
fprintf(fileID, '		             SS_OPTION_WORKS_WITH_CODE_REUSE)); */\n');
fprintf(fileID, '	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );\n');
fprintf(fileID, '}\n');
fprintf(fileID, '\n');
fprintf(fileID, 'static void mdlSetInputPortDimensionInfo(SimStruct        *S, \n');
fprintf(fileID, '                                         int_T            port,\n');
fprintf(fileID, '                                         const DimsInfo_T *dimsInfo)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;\n');
fprintf(fileID, '}\n');
fprintf(fileID, '\n');
fprintf(fileID, 'static void mdlSetOutputPortDimensionInfo(SimStruct        *S, \n');
fprintf(fileID, '                                          int_T            port, \n');
fprintf(fileID, '                                          const DimsInfo_T *dimsInfo)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '    if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;\n');
fprintf(fileID, '}\n');
fprintf(fileID, '\n');
fprintf(fileID, 'static void mdlSetInputPortFrameData(SimStruct  *S, \n');
fprintf(fileID, '                                     int_T      port,\n');
fprintf(fileID, '                                     Frame_T    frameData)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '    ssSetInputPortFrameData(S, port, frameData);\n');
fprintf(fileID, '}\n');
fprintf(fileID, '\n');
fprintf(fileID, '/* Function: mdlInitializeSampleTimes =========================================\n');
fprintf(fileID, ' * Abstract:\n');
fprintf(fileID, ' *    Specifiy  the sample time.\n');
fprintf(fileID, ' */\n');
fprintf(fileID, 'static void mdlInitializeSampleTimes(SimStruct *S)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);\n');
fprintf(fileID, '    ssSetOffsetTime(S, 0, 0.0);\n');
fprintf(fileID, '}\n');
fprintf(fileID, '\n');
fprintf(fileID, 'static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '    ssSetInputPortDataType( S, port, dType);\n');
fprintf(fileID, '}\n');
fprintf(fileID, '\n');
fprintf(fileID, 'static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '    ssSetOutputPortDataType(S, port, dType);\n');
fprintf(fileID, '}\n');
fprintf(fileID, '\n');
fprintf(fileID, 'static void mdlSetDefaultPortDataTypes(SimStruct *S)\n');
fprintf(fileID, '{\n');

for i=1:self.numParams
    fprintf(fileID, '    ssSetInputPortDataType( S, %u, SS_DOUBLE);\n',i-1);
end

% Info fields are optional
if (isfield(self.default_codeoptions,'showinfo') && self.default_codeoptions.showinfo)
    for i=1:numel(self.outputSize)+4
        fprintf(fileID, '    ssSetOutputPortDataType(S, %u, SS_DOUBLE);\n',i-1);
    end
else
    for i=1:numel(self.outputSize)
        fprintf(fileID, '    ssSetOutputPortDataType(S, %u, SS_DOUBLE);\n',i-1);
    end
end
fprintf(fileID, '}\n');
fprintf(fileID, '\n');
fprintf(fileID, '/* Function: mdlOutputs =======================================================\n');
fprintf(fileID, ' *\n');
fprintf(fileID, ' */\n');
fprintf(fileID, 'static void mdlOutputs(SimStruct *S, int_T tid)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '	int i, j, k;\n');
fprintf(fileID, '	\n');
fprintf(fileID, '	/* file pointer for printing */\n');
fprintf(fileID, '	FILE *fp = NULL;\n');
fprintf(fileID, '\n');
fprintf(fileID, '	/* Simulink data */\n');

for i=1:self.numParams
    fprintf(fileID, '	const real_T *param_%s = (const real_T*) ssGetInputPortSignal(S,%u);\n',self.paramNames{i},i-1);
end
fprintf(fileID, '	\n');

for i=1:numel(self.outputSize)
    fprintf(fileID, '    real_T *output_%s = (real_T*) ssGetOutputPortSignal(S,%u);\n',self.outputNames{i},i-1);
end
fprintf(fileID, '\n');

% Info fields are optional
if (isfield(self.default_codeoptions,'showinfo') && self.default_codeoptions.showinfo)
    fprintf(fileID, '    /* Diagnostic output ports */\n');
    fprintf(fileID, '    real_T *solver_exitflag = (real_T*) ssGetOutputPortSignal(S,%u);\n',numel(self.outputSize));
    fprintf(fileID, '    real_T *iterations = (real_T*) ssGetOutputPortSignal(S,%u);\n',numel(self.outputSize)+1);
    fprintf(fileID, '    real_T *solve_time = (real_T*) ssGetOutputPortSignal(S,%u);\n',numel(self.outputSize)+2);
    fprintf(fileID, '    real_T *primal_obj = (real_T*) ssGetOutputPortSignal(S,%u);\n',numel(self.outputSize)+3);
    fprintf(fileID, '\n');
end
    
fprintf(fileID, '\n');
fprintf(fileID, '   /* Solver data */\n');
fprintf(fileID, '	static %s_params params;\n',solverName);
fprintf(fileID, '	static %s_output output;\n',solverName);
fprintf(fileID, '	static %s_info info;\n',solverName);
if self.numSolvers == 1
    fprintf(fileID, '	int exitflag;\n');
else
    fprintf(fileID, '	int* exitflag;\n');
end

fprintf(fileID, '\n');
fprintf(fileID, '	/* Extra NMPC data */\n');
fprintf(fileID, '\n');

fprintf(fileID, '    /* Copy inputs */\n');
for i=1:self.numParams
    fprintf(fileID, '    for (i = 0; i < %u; i++) {\n',prod(self.paramSizes(i,:)));
    fprintf(fileID, '        params.%s[i] = (double) param_%s[i];\n',self.paramNames{i},self.paramNames{i});
    fprintf(fileID, '    }\n');
end
fprintf(fileID, '\n');

fprintf(fileID, '#if SET_PRINTLEVEL_%s > 0\n',solverName_constant);
fprintf(fileID, '	 /* Prepare file for printfs */\n');
fprintf(fileID, '    fp = fopen("stdout_temp","w+");\n');
fprintf(fileID, '	 if( fp == NULL ) {\n');
fprintf(fileID, '        mexErrMsgTxt("freopen of stdout did not work.");\n');
fprintf(fileID, '    }\n');
fprintf(fileID, '    rewind(fp);\n');
fprintf(fileID, '#endif\n');
fprintf(fileID, '\n');

fprintf(fileID, '	/* Call solver */\n');
fprintf(fileID, '	exitflag = %s_solve(&params, &output, &info, fp );\n',solverName);
fprintf(fileID, '\n');

fprintf(fileID, '#if SET_PRINTLEVEL_%s > 0\n',solverName_constant);
fprintf(fileID, '	/* Read contents of printfs printed to file */\n');
fprintf(fileID, '	rewind(fp);\n');
fprintf(fileID, '	while( (i = fgetc(fp)) != EOF ) {\n');
fprintf(fileID, '		ssPrintf("%%c",i);\n');
fprintf(fileID, '	}\n');
fprintf(fileID, '	fclose(fp);\n');
fprintf(fileID, '#endif\n');
fprintf(fileID, '\n');

fprintf(fileID, '	/* Copy outputs */\n');
for i=1:numel(self.outputSize)
    fprintf(fileID, '	for (i = 0; i < %u; i++){\n',prod(self.outputSize{i}));
    fprintf(fileID, '	    output_%s[i] = (real_T) output.%s[i];\n',self.outputNames{i},self.outputNames{i});
    fprintf(fileID, '	}\n');
end

% Info fields are optional
if (isfield(self.default_codeoptions,'showinfo') && self.default_codeoptions.showinfo)
    fprintf(fileID, '\n/* Diagnostic info */\n');
    if self.numSolvers == 1
        fprintf(fileID, '    iterations[0] = info.it;\n');
        fprintf(fileID, '    solve_time[0] = info.solvetime;\n');
        fprintf(fileID, '    primal_obj[0] = info.pobj;\n');
        fprintf(fileID, '    solver_exitflag[0] = exitflag;\n');
    else % multiple solvers
        % sum iterations
        fprintf(fileID, '    iterations[0] = info.it[0];\n');
        for i=2:self.numSolvers
            fprintf(fileID, ' iterations[0] += info.it[%u];\n', i-1);
        end
        fprintf(fileID, '\n');
        
        % sum solve times
        fprintf(fileID, '    solve_time[0] = info.solvetime[0];\n');
        for i=2:self.numSolvers
            fprintf(fileID, '    solve_time[0] += info.solvetime[%u];\n', i-1);
        end
        fprintf(fileID, '\n');
        
        % sum objective values
        fprintf(fileID, '    primal_obj[0] = info.pobj[0];\n');
        for i=2:self.numSolvers
            fprintf(fileID, '    primal_obj[0] += info.pobj[%u];\n', i-1);
        end
        fprintf(fileID, '\n');
        
        % Find worst exitflag (worst-to-best: -100,-10,-2,-1,0,2,1)
        if isempty(self.qcqpParams.bidx) % not branch-and-bound
            fprintf(fileID, '    solver_exitflag[0] = exitflag[0];\n');
            fprintf(fileID, '    for ( i=1; i<%u; i++) {\n', self.numSolvers);
            fprintf(fileID, '        if (exitflag[i] < solver_exitflag[0])\n');
            fprintf(fileID, '             solver_exitflag[0] = exitflag[i];\n');
            fprintf(fileID, '    };\n');
        else
            fprintf(fileID, '    solver_exitflag[0] = exitflag[0];\n');
            fprintf(fileID, '    int i = 0;\n');
            fprintf(fileID, '    for ( i=1; i<%u; i++) {\n', self.numSolvers);
            fprintf(fileID, '        if (exitflag[i] == 2 && solver_exitflag[0] == 1) {\n');
            fprintf(fileID, '             solver_exitflag[0] = 2;\n');
            fprintf(fileID, '        } else if (exitflag[i] != 2 && exitflag[i] < solver_exitflag[0]) {\n');
            fprintf(fileID, '             solver_exitflag[0] = exitflag[i];\n');
            fprintf(fileID, '        };\n');
            fprintf(fileID, '    };\n');
        end
    end
end

fprintf(fileID, '}\n');

fprintf(fileID, '\n');
fprintf(fileID, '/* Function: mdlTerminate =====================================================\n');
fprintf(fileID, ' * Abstract:\n');
fprintf(fileID, ' *    In this function, you should perform any actions that are necessary\n');
fprintf(fileID, ' *    at the termination of a simulation.  For example, if memory was\n');
fprintf(fileID, ' *    allocated in mdlStart, this is the place to free it.\n');
fprintf(fileID, ' */\n');
fprintf(fileID, 'static void mdlTerminate(SimStruct *S)\n');
fprintf(fileID, '{\n');
fprintf(fileID, '}\n');
fprintf(fileID, '#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */\n');
fprintf(fileID, '#include "simulink.c"      /* MEX-file interface mechanism */\n');
fprintf(fileID, '#else\n');
fprintf(fileID, '#include "cg_sfun.h"       /* Code generation registration function */\n');
fprintf(fileID, '#endif\n');
    
% Don't forget to close S-function file
fclose(fileID);

success = 1;

end

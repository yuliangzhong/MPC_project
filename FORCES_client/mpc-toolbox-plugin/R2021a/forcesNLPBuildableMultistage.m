classdef forcesNLPBuildableMultistage < coder.ExternalDependency
    %% FORCES PRO "mpc-toolbox-plugin" code generation utility
    
    %   Author(s): Rong Chen, MathWorks Inc.
    %
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = mfilename;
        end
        
        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end
        
        function updateBuildInfo(buildInfo, cfg)
            % forces path
            forcespath = fileparts(which('FORCESversion'));
            % solver name
            classname = mfilename;
            idx = strfind(classname,'Buildable');
            solvername = classname(1:idx-1);
            % solver header
            headerPath = [pwd filesep solvername filesep 'include'];
            isTargetPlatform = false;
            isSpeedgoat = strcmpi(cfg.getConfigProp('SystemTargetFile'), 'slrt.tlc');
            isSpeedgoatQNX = strcmpi(cfg.getConfigProp('SystemTargetFile'), 'slrealtime.tlc');
            isDSpaceMABII = strcmpi(cfg.getConfigProp('SystemTargetFile'), 'rti1401.tlc');
            isDSpaceMABXIII = strcmpi(cfg.getConfigProp('SystemTargetFile'), 'dsrt.tlc');
            isTargetPlatform = isTargetPlatform || isSpeedgoat || isSpeedgoatQNX || isDSpaceMABII || isDSpaceMABXIII;
            buildInfo.addIncludePaths(headerPath);
            % add casadi source files
            buildInfo.addSourceFiles([solvername '_casadi2forces.c']);
            buildInfo.addSourceFiles([solvername, '_casadi.c']);
            % solver library
            libPriority = '';
            libPreCompiled = true;
            libLinkOnly = true;
            try
                thisCompiler = mex.getCompilerConfigurations('C','Selected');
                settings.mexcomp.name = thisCompiler(1).Name;
                settings.mexcomp.ver = thisCompiler(1).Version;
                settings.mexcomp.vendor = thisCompiler(1).Manufacturer;
            catch
                settings.mexcomp = [];
            end
            if(isfield(settings, 'mexcomp') && isstruct(settings.mexcomp) && strncmpi(settings.mexcomp.name, 'MinGW', 5))
                isMinGW = true;
            else
                isMinGW = false;
            end
            
            if ( isSpeedgoat || isDSpaceMABII )
                libName = [solvername '.lib'];
            elseif( ismac || isunix || isMinGW || isSpeedgoatQNX || isDSpaceMABXIII )
                libName = ['lib' solvername '.a'];
            else
                libName = [solvername '_static.lib'];
            end
            if ( isTargetPlatform )
                libPath = ['$(START_DIR)' filesep solvername filesep 'lib_target'];
            else
                libPath = ['$(START_DIR)' filesep solvername filesep 'lib'];
            end
            buildInfo.addLinkObjects(libName, libPath, libPriority, libPreCompiled, libLinkOnly);
            % additional standard library
            if ispc && ~isTargetPlatform && ~isMinGW
                libPathExtra = [forcespath filesep 'libs_Intel' filesep 'win64'];
                buildInfo.addLinkObjects('libmmt.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('libirc.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('svml_dispmt.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('libdecimal.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('iphlpapi.lib', ['$(MATLAB_ROOT)' filesep 'sys' filesep 'lcc64' filesep 'lcc64' filesep 'lib64'], libPriority, libPreCompiled, libLinkOnly);
            elseif ~isTargetPlatform
                if(isMinGW)
                    buildInfo.addLinkObjects('iphlpapi.lib', ['$(MATLAB_ROOT)' filesep 'sys' filesep 'lcc64' filesep 'lcc64' filesep 'lib64'], libPriority, libPreCompiled, libLinkOnly);
                else
                    buildInfo.addLinkObjects('-lm', '', libPriority, libPreCompiled, libLinkOnly);
                end
            end
            
            
        end
        
        function [output,status,SolverInfo] = forcesNLP(coredata,onlinedata,x,lastMV)
            %#codegen
            % C interface is defined in the header file
            solvername = coredata.SolverName;
            headerName = [solvername '.h'];
            coder.cinclude(headerName);
            coder.cinclude([solvername '_casadi2forces.h']);
            % define solver input information (params, file and casadi)
            params = forcesnlmpcMultistageGetParamValues(coredata,onlinedata,x,lastMV);
            coder.cstructname(params,[solvername '_params'],'extern','HeaderFile',headerName);
            fp = coder.opaque('FILE *','NULL','HeaderFile',headerName);
            % need define extern void solvername_casadi2forces(haomin_float *x, haomin_float *y, haomin_float *l, haomin_float *p, haomin_float *f, haomin_float *nabla_f, haomin_float *c, haomin_float *nabla_c, haomin_float *h, haomin_float *nabla_h, haomin_float *hess, solver_int32_default stage, solver_int32_default iteration);
            casadi = coder.opaque([solvername '_extfunc'],['&' solvername '_casadi2forces'],'HeaderFile',headerName);
            % define solver output information (output, exitflag, info)
            output = struct('MVopt',zeros(coredata.p*coredata.nmv,1),'Xopt',zeros((coredata.p+1)*coredata.nx,1),'z',zeros(coredata.nz,1));
            coder.cstructname(output,[solvername '_output'],'extern','HeaderFile',headerName);
            if strcmp(coredata.SolverType,'SQP')
                SolverInfo = struct('it', int32(0), 'res_eq', 0, 'rsnorm', 0, 'pobj', 0, 'solvetime', 0, 'fevalstime', 0, 'QPtime', 0);
            else
                SolverInfo = struct('it', int32(0), 'it2opt', int32(0), 'res_eq', 0, 'res_ineq', 0, 'rsnorm', 0, 'rcompnorm', 0, 'pobj', 0, 'mu', 0, 'solvetime', 0, 'fevalstime', 0);
            end
            coder.cstructname(SolverInfo,[solvername '_info'],'extern','HeaderFile',headerName);
            exitflag = int32(0);
            % generate code with solver DLL/LIB
            exitflag = coder.ceval([solvername '_solve'],coder.ref(params),coder.ref(output),coder.ref(SolverInfo),fp,casadi);
            status = double(exitflag);
        end
        
    end
    
end

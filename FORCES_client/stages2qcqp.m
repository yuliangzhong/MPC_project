% Convert a FORCESPRO stages structure to QCQP input data format.
% 
%    [H,f,Aineq,bineq,Aeq,beq,l,Q,r,lb,ub] = STAGES2QCQP(STAGES,PARAMS,PROBLEM) 
%    converts the structure STAGES into a QCQP description for
%    the quadratically constrained problem
%
%       minimize 0.5*x'*H*x + f'*x
%     subject to Aineq*x <= bineq
%                Aeq*x == beq
%                x'*Q{i}*x + l(:,i)'*x <= r(i) for i=1...M
%    
%    After conversion, the problem above can be solved, for example, 
%    using QUADPROG by calling
%
%              x = quadprog(H,f,Aineq,bineq,Aeq,beq,lb,ub)
%     
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.

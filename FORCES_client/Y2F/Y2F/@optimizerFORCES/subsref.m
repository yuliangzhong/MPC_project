function [output,exitflag,info] = subsref(self,subs)
%SUBSREF Overload of subsref. Makes A{B} for object A possible.
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

if isequal(subs.type,'.')
    error('No fields accessible in OPTIMIZERFORCES.');
elseif isequal(subs.type,'{}') || isequal(subs.type,'()')  % --> call solver
    if numel(subs.subs) == 1 && isa(subs.subs{1},'cell')
        paramValues = subs.subs{1};
    else
        paramValues = subs.subs;
    end
    [output,exitflag,info] = self.interfaceFunction(paramValues);
end
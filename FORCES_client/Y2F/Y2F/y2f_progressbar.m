function status = y2f_progressbar(status,i,N,width)
% Displays progress bar in command window
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

bar = repmat(sprintf('='),1,round(i/N*width));
msg = sprintf('[%-30s] %d/%d', bar, i, N);
fprintf([status, msg]);
status = repmat(sprintf('%c',8), 1, length(msg));

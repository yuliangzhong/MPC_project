function f = objective ( X )
y = X(2);
F = X(5);
s = X(6);
slack = X(7);
% Want to minimize fuel, but maximize progress along the y direction
f = -100*y + 0.1*F^2 + 0.01*s^2 + 0.5*slack^2;
function v = inequalities( X )

x=X(1);
y=X(2);
slack = X(7);
% at any point in time, there is a minimum and maximum radius for the car
v = x^2 + y^2 - slack^2;
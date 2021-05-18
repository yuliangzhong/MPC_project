# Y2F - YALMIP to FORCES PRO Interface

This project provides a simple MATLAB interface that connects [YALMIP](http://users.isy.liu.se/johanl/yalmip/pmwiki.php?n=Main.WhatIsYALMIP)
with [FORCES PRO](https://www.embotech.com/FORCES-Pro). It combines YALMIP's intuitiveness with the high efficiency of FORCES PRO for rapid development.

## Installation

Simply download the code to the desired location and add the `Y2F` folder to your [MATLAB search path](http://ch.mathworks.com/help/matlab/ref/addpath.html). 

The Y2F interface requires a working YALMIP installation. See [https://yalmip.github.io/tutorial/installation/](https://yalmip.github.io/tutorial/installation/) for instructions on how to install YALMIP.

The code has been tested with YALMIP release 20150919. Older versions might work but have not been tested.

## Example Usage

Consider the following linear MPC problem with lower and upper bounds on state and inputs, and a terminal cost term:

![\begin{aligned}\text{minimize} \quad & x_N^T P x_N + \sum_{i=0}^{N-1} x_i^T Q x_i + u_i^T R u_i \\ \text{s.t.} \quad & x_0 = x(t) \\& x_{i+1} = Ax_i + Bu_i \\& \underline{x} \leq x_i \leq \bar{x} \\& \underline{u} \leq u_i \leq \bar{u}\end{aligned}](example_problem.png)

This problem is parametric in the initial state x(t), and the first input u0 is typically applied to the system after a solution has been obtained. The following code generates a solver that returns u0, which can then be applied to the system:

```
% Define variables
X = sdpvar(nx,N+1,'full'); % state trajectory: x0,x1,...,xN (columns of X)
U = sdpvar(nu,N,'full'); % input trajectory: u0,...,u_{N-1} (columns of U)

% Initialize objective and constraints of the problem
cost = 0; const = [];

% Assemble MPC formulation
for i = 1:N        
    % cost
    if( i < N )
        cost = cost + 0.5*X(:,i+1)'*Q*X(:,i+1) + 0.5*U(:,i)'*R*U(:,i);
    else
        cost = cost + 0.5*X(:,N+1)'*P*X(:,N+1) + 0.5*U(:,N)'*R*U(:,N);
    end
    
    % model
    const = [const, X(:,i+1) == A*X(:,i) + B*U(:,i)];

    % bounds
    const = [const, umin <= U(:,i) <= umax];
    const = [const, xmin <= X(:,i+1) <= xmax];
end

controller = optimizerFORCES(const, cost, codeoptions, X(:,1), U(:,1));
```

The generated solver can then be called using curly braces:

```
u0 = controller{x0};
```

## Limitations

- The Y2F interface only supports convex quadratically constrained programs (QCQPs). FORCES' NLP solver is currently not supported.
- If you have rate constraints or cost, you need to manually define delta variables (e.g. U+ = U + dU, and then constrain dU). Otherwise, Y2F cannot figure out the multistage structure from your problem, and you might obtain a slow solver.


## License

The code is licensed under the MIT License. For more information see LICENSE file.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) file.

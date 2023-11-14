clear all;
clc;

%% System modeling
dt = 0.1;
T  = 100;
d = 50;
m = 1000;
a = 1 - d/m*dt;
b = dt/m;

%% Kalman
P = 100;
C = 1;
Q = 0;
R = 1;

%% Constraint
xlow  = -10;
ulow  = 0;
xhigh = 100;
uhigh = 10000;

lb = [xlow*ones(4,1); ulow*ones(4,1)];
ub = [xhigh*ones(4,1); uhigh*ones(4,1)];

%% Weights settings
weight_sets = {[10000, 0.001], [100, 1], [1000,0.01], [100000,0.0001]}; % Define different sets of weights as a cell array

%% Input reference
xref = 20;

%% Simulation loop for different weights
for w = 1:length(weight_sets)
    q = weight_sets{w}(1);
    r = weight_sets{w}(2);
    
    % Redefine G and f based on new weights
    G = [q*eye(4), zeros(4); zeros(4), r*eye(4)];
    f = [-q*xref*ones(4,1);zeros(4,1)];
    
    %% Initialization for this run
    x0     = 0;
    xArray = [];
    yArray = [];
    
    Aeq = [1 0 0 0 -b 0 0 0;
           -a 1 0 0 0 -b 0 0;
           0 -a 1 0 0 0 -b 0;
           0 0 -a 1 0 0 0 -b];
    
    %% Simulation
    for i = 1:T
        beq = [a*x0; zeros(3,1)];
        x   = quadprog(G, f, [], [], Aeq, beq, lb, ub);
        y   = a*x0 + b*x(5) + sqrt(R)*randn; % output
        
        % Kalman predict
        x0  = a*x0 + b*x(5);
        P   = a*P*a' + Q;
        
        % Kalman update
        K   = P*C'/(C*P*C' + R);
        x0  = x0 + K*(y - C*x0);
        P   = (eye(1) - K*C)*P;
        
        % Append to xArray and yArray
        xArray = [xArray x];
        yArray = [yArray y];
    end
    
    %% Plotting for this set of weights
    figure(w);
    subplot(2,1,1)
    plot(dt:dt:T*dt, xArray(1,:), 'o:')
    title(sprintf('State with weights q = %d, r = %d', q, r))
    ylim([0 25])
    grid on;
    subplot(2,1,2)
    stairs(dt:dt:T*dt, xArray(5,:), 'o-')
    title(sprintf('Control with weights q = %d, r = %d', q, r))
    grid on;
end

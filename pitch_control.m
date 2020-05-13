%% Initialization
clear;
clc;
close all;

dt = 0.1;             % Time step
tf = 2;                % Simulation time 
t  = 0:dt:tf;          % Time step vector

% MPC variables:
theta_ref = 20;%deg2rad(20); % pitch reference
N = 10; % time horizon

% Model variables:
F_model = eye(3)+[-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0]*dt;  % F = A
G_model = [0.232; 0.0203; 0]*dt;                                  % G = B
H_sensor = [0 0 1];                                                % H = C

% Boeing variables
max_deflection_angle = 20;%deg2rad(20);
min_deflection_angle = -max_deflection_angle;
delta_theta_max = (max_deflection_angle - min_deflection_angle) / (2 / dt);

% Kalman variables
Q_kalman =eye(3)* 0.01; % Model uncertainty
R_kalman = 0.01;        % Sensor uncertainty

x    = [0;0;0];         % Initial state
xhat = [0;0;0];         % State estimate variable
Pplus = eye(3);         % Estimated model uncertainty

% Optimization variables:
H_cost_small = [0 0 0;0 0 0;0 0 2]; % H matrix for single timestep cost function
H_cost = zeros(4*N,4*N); % H matrix for cost function

for i=1:3:3*N
    H_cost(i:i+2, i:i+2) = H_cost_small;
end

Aeq = zeros(3*N, 4*N);

Aeq(1:3,1:3) = eye(3,3);
Aeq(1:3,4+3*(N-1)) = -G_model;

for i=4:3:3*N
    Aeq(i:i+2,i-3:i-1) = -F_model;
    Aeq(i:i+2,i:i+2) = eye(3,3);
    Aeq(i:i+2,3*N+(i+2)/3) = -G_model;
end

lb = [ones(3*N,1) * (-inf);ones(N,1) * min_deflection_angle];
ub = [ones(3*N,1) * inf;ones(N,1) * max_deflection_angle];

Aineq = [zeros((N-1)*2,N)];

for i=1:N-1
    Aineq(i*2-1,i) = -1;
    Aineq(i*2-1,i+1) = 1;
    
    Aineq(i*2-1+1,i) = 1;
    Aineq(i*2-1+1,i+1) = -1;
end

Aineq = [zeros((N-1)*2,3*N) Aineq];

bineq = ones((N-1)*2,1) * delta_theta_max;


%% MPC

u_vec = zeros(size(t));
x_vec = [];

for index = 0:size(t,2)-1
    % Open loop control prediction
    [x_N, u_N, u] = open_loop_control(N, theta_ref, H_cost, Aeq, F_model, Aineq, bineq, xhat, lb, ub);

    % Get kalman state estimate
    [xhat,x,Pplus] = KF_function(F_model,G_model,H_sensor,Q_kalman,R_kalman,u,dt,x,xhat,Pplus);
    
    % Get plotting data
    u_vec(index+1) = u;
    x_vec = [x_vec xhat];
end

subplot(4,1,1);
plot(t,x_vec(1,:));

subplot(4,1,2);
plot(t,x_vec(2,:));

subplot(4,1,3);
plot(t,x_vec(3,:));

subplot(4,1,4);
plot(t,u_vec);









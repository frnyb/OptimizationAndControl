%% Initialization
clear;
clc;
close all;

dt = 0.1;             % Time step
tf = 50;                % Simulation time 
t  = 0:dt:tf;          % Time step vector

% MPC variables:
theta_ref = 20;%deg2rad(20); % pitch reference
N = 30; % time horizon

% Model variables:
F_model = eye(3)+[-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0]*dt;  % F = A
G_model = [0.232; 0.0203; 0]*dt;                                  % G = B
H_sensor = [0 0 1];                                               % H = C

% Boeing variables
max_deflection_angle = 20;%deg2rad(20);
min_deflection_angle = -max_deflection_angle;
delta_theta_max = (max_deflection_angle - min_deflection_angle) / (5 / dt);

% Kalman variables
Q_kalman =eye(3)* 0.01; % Model uncertainty
R_kalman = 100;        % Sensor uncertainty

x    = [0;0;0];         % Initial state
xhat = [0;0;0];         % State estimate variable
Pplus = eye(3);         % Estimated model uncertainty

% Optimization variables:
H_cost_small = [0 0 0;0 0 0;0 0 2]; % H matrix for single timestep cost function
H_cost = zeros(4*N,4*N); % H matrix for cost function

for i=1:3:3*N
    H_cost(i:i+2, i:i+2) = H_cost_small;
end

% H_cost(3*N+1:size(H_cost,1),3*N+1:size(H_cost,2)) = eye(size(N));

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

Atemp = zeros(2, 4*N);
Atemp(1, 3*N+1) = 1;
Atemp(2, 3*N+1) = -1;

Aineq = [Atemp;Aineq];

bineq = ones(2*N,1) * delta_theta_max;

%% MPC

u_vec = [];
x_hat_vec = [];
x_vec = [];
y_vec = [];
x_N_vec = [];

u = 0;

for index = 0:size(t,2)-1
    if index*dt == 25
        theta_ref = -10;
    end
    % Update bineq
    bineq(1) = delta_theta_max + u;
    bineq(2) = delta_theta_max - u;
    
    % Open loop control prediction
    [x_N, u_N, u] = open_loop_control(N, theta_ref, H_cost, Aeq, F_model, Aineq, bineq, xhat, lb, ub);

    % Get kalman state estimate
    [xhat,x,Pplus,y] = KF_function(F_model,G_model,H_sensor,Q_kalman,R_kalman,u,dt,x,xhat,Pplus);
    
    % Get plotting data
    u_vec = [u_vec u_N];
    x_hat_vec = [x_hat_vec xhat];
    x_vec = [x_vec x];
    y_vec = [y_vec y];
    x_N_vec = [x_N_vec x_N(:,3)];
end

plot(t,x_vec(3,:));
hold on;
plot(t,x_hat_vec(3,:));
hold on;
plot(t,y_vec);
hold off;
legend('True theta','Estimated theta','Measured theta');
title('Kalman filter');
xlabel('Time [s]');
ylabel('Theta [deg]');

figure(2);

subplot(2,1,1);
plot(t,u_vec(1,:),'.');
legend('elevator deflection angle control input');
ylabel('[deg]');
title('Control Inputs');

subplot(2,1,2);
plot(t,x_vec(3,:));
legend('true theta');
ylabel('[deg]');
xlabel('Time [s]');
title('True Theta');

figure(3);
subplot(2,1,1);

for i = 24/dt:34/dt
    plot(t(i:i+9),u_vec(1:10,i),'.-','HandleVisibility','off');
    hold on;
end
plot(t(24/dt:34/dt),u_vec(1,24/dt:34/dt),'o');
hold off;
legend('MPC implemented control inputs');
title('MPC Implemented and Simulated Control Inputs');
ylabel('[deg]');

subplot(2,1,2);

for i=24/dt:34/dt
    plot(t(i:i+9),x_N_vec(1:10,i),'.-','HandleVisibility','off');
    hold on;
end
plot(t(24/dt:34/dt),x_hat_vec(3,24/dt:34/dt),'o');
hold off;
legend('theta Kalman estimate');
title('Kalman Estimated Theta and MPC Simulated Theta');
ylabel('[deg]');
xlabel('Time [s]');







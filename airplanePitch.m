%% Initialization
clear;
clc;
close all;

dt = 0.001;             % Time step
tf = 10;                % Simulation time 
t  = dt:dt:tf;          % Time step vector

theta_ref = 10; % pitch reference
N = 10; % time horizon
x_0 = [0;0;0];

F_model = eye(3)+[-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0]*dt;  % F = A
G_model = [0.232; 0.0203; 0]*dt;                                  % G = B
H_sensor = [0 0 1];                                                % H = C

u = [1];              % Control signal

% Optimization specific matrices:

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

beq = [F_model * x_0;zeros(3*N-3,1)];


%% error covariance matrix
Q =eye(3)* 0.01;
R = 0.01;

% initial data
x    = [0;0;1];
xhat = [0;0;1];
Pplus = eye(3);

% for plotting
xArray    = [];
xhatArray = [];

for i=1:tf/dt
    
    xArray    = [xArray x];
    xhatArray = [xhatArray xhat];

    % Simulate the system
    x = F*x + G*u + sqrt(Q)*[randn randn randn]'*dt;
    y = H*x + sqrt(R)*randn*dt;

    % Prediction
    xmin  = F*xhat + G*u;
    Pmin  = F*Pplus*F' + Q;
    % Update
    Sigma = inv(H*Pmin*H' + R);
    K     = Pmin*H'*Sigma;
    Pplus = (eye(3)-K*H)*Pmin;
    xhat  = xmin + K*(y-H*xmin);
 
end

%% Plotting
figure(1);
subplot(3,1,1)
plot(t,xArray(1,:), 'k')
hold on
plot(t,xhatArray(1,:), 'r:')
grid on;
grid minor
ylabel('x_1')
set(gca)
h1 = legend('True State','Estimated State');
subplot(4,1,2)
plot(t,xArray(2,:), 'k')
hold on
plot(t,xhatArray(2,:), 'r:')
grid on;
grid minor
ylabel('x_2')
set(gca)
subplot(4,1,3)
plot(t,xArray(3,:), 'k')
hold on
plot(t,xhatArray(3,:), 'r:')
grid on;
grid minor
ylabel('x_3')
set(gca)
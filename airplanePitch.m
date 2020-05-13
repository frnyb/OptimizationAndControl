% This is an example of Kalman Filter application airplane pitch

clear;
clc;
close all;

dt = 0.001;
tf = 10;
t   = dt:dt:tf;

% system description


F = eye(3)+[-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0]*dt ;
G = [0.232; 0.0203; 0]*dt;
H = [0 0 1]; 

u = 1;

% error covariance matrix
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
function [xhat,x,Pplus,y]=KF_function(F,G,H,Q,R,u,dt,x_init,xhat_init,Pplus_init)
 
    
    %----INPUT----%
    
%F = Is the A matrix in state-space model
%G = Is the B matrix in state-space model
%H = Is the C matrix in state-space model
%Q = Is the the noise in the model(state-space)
%R = Is the noise in the measurements
%x_init = Needs to be set
%u = Is the control input to the state-space model
%dt = Is the time step for the state-space model
    

    %----OUTPUT----%
    
% x = Is the output from the system is simulated
% xhat = Is the output from the KF. The estimation data.
% Pplus = Is the term that makes sure the prediction gets better in the futur


    %----Set values----%
    
Noise_Q=ones(size(Q,1),1)*randn; % This is Nx1 matrix where Q= NxN

Noise_R=ones(size(R,1),1)*randn; % This is 1x1

%The inits
x=x_init;
xhat=xhat_init;
Pplus=Pplus_init;


    % Simulate the system
    x = F*x + G*u + sqrt(Q)*Noise_Q*dt;
    y = H*x + sqrt(R)*Noise_R*dt;           %Noise_R size can be the reason to it fails, if it does.

    % Prediction
    xmin  = F*xhat + G*u;
    Pmin  = F*Pplus*F' + Q;
    % Update
    Sigma = inv(H*Pmin*H' + R);
    K     = Pmin*H'*Sigma;
    Pplus = (eye(3)-K*H)*Pmin;
    xhat  = xmin + K*(y-H*xmin);
end


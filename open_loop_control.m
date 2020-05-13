function [x_N, u_N, u_0] = open_loop_control(N, theta_ref, H_cost, Aeq, F_model, x, lb, ub)

f_cost_small = [0;0;-2*theta_ref];
f_cost = zeros(4*N,1);

for i = 1:3:3*N
    f_cost(i:i+2) = f_cost_small;
end

beq = [F_model * x;zeros(3*N-3,1)];

z = quadprog(H_cost, f_cost, [], [], Aeq, beq, lb, ub);

x_N = zeros(N,3);
for i=0:3:3*N-1
    x_N((i/3)+1,:) = z(i+1:i+3);
end

u_N = z(3*N+1:size(z,1));
u_0 = u_N(1);
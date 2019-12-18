function dx = mpc_dynamics(t,x0,N,dt,x_wp,t_wp,xf,u_ff,K)

% Forward simulate to get initial trajectory
[x_bar,u_bar] = forwardInt(x0, u_ff, K, N,dt, xf);

t_wp = t_wp - t;
[x_bar,K,u_ff] = slqSolve(x_bar,u_bar,N,dt,x0, x_wp,t_wp,xf);

dx = zeros(12,1);
dx(1:6) = x0(7:12);

% tvec = 0:dt:N;
% u_ff = interp(tvec, u_ff, t)';
% x_bar = interp(tvec, x_bar, t)';
% K = reshape(interp1(tvec, reshape(K,numel(K(:,:,1)),N)', t),2,2)
u = u_ff(:,1) + K(:,:,1)*(x0 - x_bar);

dx(7:12) = u;
dx(9) = dx(9) - 9.81;




function dx = dynamics(t,x, tspan)
dx = zeros(4,1);

q = x(1:2);
dq = x(3:4);
u = [0;0];

dx(1:2) = dq;

[M,C,N,Y] = computeDynamicMatrices(q,dq,u);

dx(3:4) = M\(Y-C*dq-N);

end
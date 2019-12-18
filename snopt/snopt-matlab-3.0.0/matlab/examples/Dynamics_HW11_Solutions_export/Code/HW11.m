close all;clear;clc;

x0 = [0;0;0;0];
u0 = [0;0];

dt = 0.02;
tspan = 0:dt:1.5;
N = length(tspan);
vars = [x0;u0];

% % Code to produce initial dynamically feasible trajectory with ode45
% [t,x] = ode45(@dynamics,tspan,x0, [], tspan);
% X0 = [x,u0'.*ones(N,length(u0))];

X0 = zeros(N,numel(vars));

options = optimoptions('fmincon','Display', 'iter', 'MaxFunctionEvaluations', 1e5);
[sol, cost] = fmincon(@minTorque, X0, [],[],[],[],[],[],@nonlincon, options);

x = sol(:,1:4);
animateHW11(x,dt);
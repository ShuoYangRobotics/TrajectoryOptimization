
addpath ../../
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Parameters for the dynamics function                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
dyn.m1 = 1;  % elbow mass
dyn.m2 = 1; % wrist mass
dyn.m3 = 0.3; % wheel mass
dyn.g = 9.81;  % gravity
dyn.l1 = 0.5;   % length of first link
dyn.l2 = 0.5;   % length of second link
dyn.miu = 0.4;
dyn.xc = 0.1;
dyn.yc = -0.95;
dyn.r  = 0.3;

t0 = 0;
tF = 1;  %For now, force it to take exactly this much time.
q0 = [pi/6;-pi/2;0];   %[q1;q2;q3];  %initial angles   %Stable equilibrium
dq0 = [1e-6;1e-6;1e-6];   %[dq1;dq2;dq3];  %initial angle rates

qF = [2.7*pi/6;-pi/2.7;pi/5];  %[q1;q2;q3];  %final angles    %Inverted balance
dqF = [1e-6;1e-6;1e-6];  %[dq1;dq2;dq3];  %final angle ratesx

x0 = [q0;dq0];
xF = [qF;dqF];

%%
t = 0;
drawFlickingFingerState(t,x0,xF,dyn)
%%
x0 = [q0;dq0];
xF = [qF;dqF];
dt = 0.01;
time = t0:dt:tF;
nGrid = length(time);  % depends on timestep
nState = 6;            % two joint + one wheel
nControl = 2;          % two actuation
nContactForce = 4;     % lambda_x  lambda_x-  lambda_z slack
nZ = nState*nGrid + nControl*nGrid + nContactForce*nGrid;  % dimension of decision variable
stateIdx = 1:nState*nGrid;
controlIdx = nState*nGrid+1:nState*nGrid + nControl*nGrid;
forceIdx = nState*nGrid+nControl*nGrid+1:nZ;

% load zSoln from a dataset 

% load('clkwheelrotate.mat')
load('counterclkwheelrotate.mat')
% load('nowheelrotate.mat')

xSoln = [reshape(zSoln(stateIdx),nState,nGrid);
         reshape(zSoln(controlIdx),nControl,nGrid);
         reshape(zSoln(forceIdx),nContactForce,nGrid)];
A.plotFunc = @(t,z)( drawFlickingFinger(t,z,dyn,x0,xF) );
A.speed = 0.03;
A.figNum = 101;
animate(time(:,1:end-1),xSoln(:,1:end-1),A)
%MAIN.m  --  solve swing-up problem for acrobot
%
% This script finds the minimum torque-squared trajectory to swing up the
% acrobot robot: a double pendulum with a motor between the links
%
%
%%
clc; clear;
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
dq0 = [0;0;0];   %[dq1;dq2;dq3];  %initial angle rates

qF = [2.7*pi/6;-pi/2.7;pi/5];  %[q1;q2;q3];  %final angles    %Inverted balance
dqF = [0;0;0];  %[dq1;dq2;dq3];  %final angle ratesx

x0 = [q0;dq0];
xF = [qF;dqF];

% test functions 
testZ = [0;pi/2;pi/4;0;0;0;0;0;0;0;0;0]; 
t = 0;
drawFlickingFinger(t,testZ,dyn,x0,xF)
M = computeM(testZ,dyn)
C = computeC(testZ,dyn)
G = computeG(testZ,dyn)
J = computeJ(testZ,dyn)
fk = computeFK(testZ,dyn)
phi = wheelConstraint(testZ,dyn)
dz = flickingFingerDynamics(testZ,[0;0],[-1;0;1],dyn)

Jb = computeJb(testZ,dyn)
Jb*[1;1]


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
%%


% construct a initial value
% zInit = zeros(nZ,1);
% nowheelxF = xF; nowheelxF(3) = x0(3);
% guess_mid = [2.7*pi/6;-pi;x0(3);0;0;0];
% 
% linearInterpolate = interp1([0 0.5 1],[x0 guess_mid xF]',time);
% zInit(stateIdx) = linearInterpolate';
% % simple strategy for calculating control
% % for i = 1:nGrid
% %     zInit(nState*nGrid+1+(i-1)*nControl:nState*nGrid+1+(i-1)*nControl+1) = flickingFingerForwardDynamics(linearInterpolate(:,i),dyn);
% % end

% %% construct a initial value using previous no wheel rotation trajectory
load('counterclkwheelrotate_different_init.mat')
zInit = zSoln;

% construct a fmincon problem 
problem.objective = @(z) (robotObj(dt,z,x0,xF,nGrid,nState,nControl,nContactForce));
problem.x0 = zInit;
problem.Aineq = [];problem.bineq = [];problem.Aeq = [];problem.beq = [];
problem.lb = [];
problem.ub = [];
problem.solver = 'fmincon';
% % problem.options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations', 1e5);
problem.options = optimoptions('fmincon','Display','iter','OptimalityTolerance', 1e-4,'MaxFunctionEvaluations', 2e5,'SpecifyConstraintGradient',true);
% problem.nonlcon = @(z) (robotConstraint_hermitesimpson(dt,reshape(z(stateIdx),nState,nGrid), reshape(z(controlIdx),nControl,nGrid),@robotDynamics,x0,xF));
problem.nonlcon = @(z) (robotConstraint_trapazoid(dt,z,x0,xF,nGrid,nState,nControl,nContactForce,dyn));

tic;
[zSoln, objVal,exitFlag,output] = fmincon(problem);
nlpTime = toc

% xSoln = reshape(zSoln,nState+nControl+nContactForce,nGrid);
xSoln = [reshape(zSoln(stateIdx),nState,nGrid);
         reshape(zSoln(controlIdx),nControl,nGrid);
         reshape(zSoln(forceIdx),nContactForce,nGrid)];
A.plotFunc = @(t,z)( drawFlickingFinger(t,z,dyn,x0,xF) );
A.speed = 0.1;
A.figNum = 101;
animate(time(:,1:end-1),xSoln(:,1:end-1),A)
save('counterclkwheelrotate_different_init.mat','zSoln','objVal')
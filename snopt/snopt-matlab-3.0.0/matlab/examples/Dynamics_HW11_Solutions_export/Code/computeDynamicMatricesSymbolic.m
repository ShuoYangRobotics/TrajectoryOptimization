clear;clc;

% Declare symbolic variables
syms q1 q2 dq1 dq2 real
syms tau_1 tau_2 real

% Group syms into vectors
q = [q1;q2];
dq = [dq1;dq2];
tau = [tau_1;tau_2];

% Declare system parameters
m = 1;
L = 1;
g = 9.81;

%% Problem 1.3

% Compute generalized inertia matrices for each link
M1 = [m 0 0;
    0 m 0;
    0 0 1/12*m*L^2];
M2 = M1;

% Compute body Jacobians for each link
Jb_sL1 = [0 0;
    L/2 0
    1 0];
Jb_sL2 = [L*sin(q2) 0
    L*cos(q2)+L/2 L/2
    1 1];

% Compute manipulator inertia tensor
M = simplify(Jb_sL1'*M1*Jb_sL1 + Jb_sL2'*M2*Jb_sL2);

% % Compute Coriolis matrix (Uncomment below to evaluate, takes some time)
C  = sym(zeros(length(q),length(q)));
for ii = 1:length(q)
    for jj = 1:length(q)
        for kk = 1:length(q)
            C(ii,jj) = C(ii,jj) + 1/2*(diff(M(ii,jj),q(kk)) + diff(M(ii,kk),q(jj)) - diff(M(jj,kk),q(ii)))*dq(kk);
        end
    end
end
C = simplify(C);

% Compute nonlinear and applied force terms (no constraints)
V = simplify(m*g*L/2*sin(q1) + m*g*(L*sin(q1) + L/2*sin(q1+q2)));
N = jacobian(V, q)';
Y = [tau_1;tau_2];

% Export as Matlab Function
matlabFunction(M,C,N,Y, 'File', 'computeDynamicMatrices', 'Outputs', {'M','C','N','Y'}, 'Vars', {q,dq,tau});


syms sqke1 sqke2 sqke3 sdqke1  sdqke2 sdqke3 sqk1e1 sqk1e2 sqk1e3  sdqk1e1 sdqk1e2 sdqk1e3 real
syms u1k1 u2k1 real
syms c1k c2k c3k slackk real
syms c1k1 c2k1 c3k1 slackk1 real
empty = sym('empty','real');

% need to derive again when these changes
p.m1 = 1;  % elbow mass
p.m2 = 1; % wrist mass
p.m3 = 0.3; % wheel mass
p.g = 9.81;  % gravity
p.l1 = 0.5;   % length of first link
p.l2 = 0.5;   % length of second link
p.miu = 0.4;
p.xc = 0.1;
p.yc = -0.95;
p.r  = 0.3;
dt = 0.01;


state = [sqke1;sqke2;sqke3;
         sdqke1;sdqke2;sdqke3;
         sqk1e1;sqk1e2;sqk1e3;
         sdqk1e1;sdqk1e2;sdqk1e3;
         u1k1;u2k1;
         c1k;c2k;c3k;slackk;
         c1k1;c2k1;c3k1;slackk1];

sqk = [sqke1; sqke2; sqke3];
sdqk = [sdqke1; sdqke2; sdqke3]; 
sqk1 = [sqk1e1; sqk1e2; sqk1e3];
sdqk1 = [sdqk1e1; sdqk1e2; sdqk1e3];


M = computeM([sqk1;sdqk1],p);
C = computeC([sqk1;sdqk1],p);
G = computeG([sqk1;sdqk1],p);
Jb = computeJb([sqk1;sdqk1],p);

[contactAnglek1,~] = contactSpeedAngle([sqk1;sdqk1],p);
[contactAnglek,diffV] = contactSpeedAngle([sqk;sdqk],p);
% rotate lambda force represented in contact frame to world frame
lambda_ck1 = [c1k1-c2k1;c3k1]; 
lambda_wk1 = [cos(contactAnglek1)  sin(contactAnglek1);
             -sin(contactAnglek1)  cos(contactAnglek1)]*lambda_ck1;

lambda_tk1 = [[cos(sqk1(1)+sqk1(2))  sin(sqk1(1)+sqk1(2));
              -sin(sqk1(1)+sqk1(2))   cos(sqk1(1)+sqk1(2))]*lambda_wk1;0];

phi = wheelConstraint([sqk;sdqk],p);

% diffV > 0 means the finger moves positive direction in reference frame
% then the force should be c1k = 0, c2k = 1
% then finger experience [-1;c3] force

             % sqk-sqk1+dt*sdqk1 
             % 12-11 I thought Prof Posa made a mistake here, then I found
             % it it correct
ceq_part = [sqk-sqk1+dt*sdqk1;  %dim3
            M*(sdqk1(1:2)-sdqk(1:2))+dt*(C*sdqk1(1:2)+G-[u1k1;u2k1]-Jb'*(lambda_tk1)); %dim2
            (1/2*p.m3*p.r^2)*(sdqk1(3)-sdqk(3))-dt*(lambda_ck1(1)*p.r) %dim1
            phi*c3k;
            % 12-11 I thought this should be c3k-c1k-c2k, later I found it
            % is correct
            (p.miu*c3k-c1k-c2k)*slackk;
            (slackk+diffV)*c1k;
            (slackk-diffV)*c2k];  
        
c_part = [-phi;                                                            %dim1
          -c1k;
          -c2k;
          -c3k;  
          -slackk;%dim4
          -(p.miu*c3k-c1k-c2k);  %dim1
          -(slackk+diffV);                                     %dim1
          -(slackk-diffV)]; %dim1
      

[ceq, ceqi, ceqz, ceqzi, ceqzd] = computeGradients(ceq_part,state,empty); 
[c, ci, cz, czi, czd] = computeGradients(c_part,state,empty);  

% Write function file:
matlabFunction(ceq, ceqi,...   %dynamics
    ceqz, ceqzi, ceqzd, ...  %gradients
    'file','autoGen_ceq_grad.m',...
    'vars',{...
    'sqke1', 'sqke2', 'sqke3'...
    'sdqke1', 'sdqke2', 'sdqke3'...
    'sqk1e1', 'sqk1e2', 'sqk1e3'...
    'sdqk1e1', 'sdqk1e2', 'sdqk1e3'...
    'u1k1','u2k1',...
    'c1k', 'c2k', 'c3k', 'slackk',...
    'c1k1', 'c2k1', 'c3k1', 'slackk1',...
    'empty'});
matlabFunction(c, ci,...   %dynamics
    cz, czi, czd, ...  %gradients
    'file','autoGen_c_grad.m',...
    'vars',{...
    'sqke1', 'sqke2', 'sqke3'...
    'sdqke1', 'sdqke2', 'sdqke3'...
    'sqk1e1', 'sqk1e2', 'sqk1e3'...
    'sdqk1e1', 'sdqk1e2', 'sdqk1e3'...
    'u1k1','u2k1',...
    'c1k', 'c2k', 'c3k', 'slackk',...
    'c1k1', 'c2k1', 'c3k1', 'slackk1',...
    'empty'});

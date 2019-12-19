syms sqke1 sqke2 sqke3 sdqke1  sdqke2 sdqke3 sqk1e1 sqk1e2 sqk1e3  sdqk1e1 sdqk1e2 sdqk1e3 real
syms u1k u2k u1k1 u2k1 real
syms c1k c2k c3k slackk real
syms c1k1 c2k1 c3k1 slackk1 real
syms dt real
empty = sym('empty','real');

state = [sqke1;sqke2;sqke3;          %3
         sdqke1;sdqke2;sdqke3;       %3
         sqk1e1;sqk1e2;sqk1e3;       %3
         sdqk1e1;sdqk1e2;sdqk1e3;    %3
         u1k;u2k;                    %2
         u1k1;u2k1;                  %2
         c1k;c2k;c3k;slackk;         %4
         c1k1;c2k1;c3k1;slackk1];    %4
     
     
J_part = 0.0001*0.5*dt*(u1k^2+u2k^2+u1k1^2+u2k1^2);

% force_z_diff = ((c3k1-c3k)/dt);
% force_x_diff = (((c1k1-c2k1)-(c1k-c2k))/dt);

J_part = J_part + 0.01*force_z_diff*force_z_diff + 0.01*force_x_diff*force_x_diff;


[jeq, jeqi, jeqz, jeqzi, jeqzd] = computeGradients(J_part, state, empty); 

matlabFunction(jeq, jeqi,...   %dynamics
    jeqz, jeqzi, jeqzd, ...  %gradients
    'file','autoGen_J_grad.m',...
    'vars',{...
    'dt','sqke1', 'sqke2', 'sqke3'...
    'sdqke1', 'sdqke2', 'sdqke3'...
    'u1k','u2k',...
    'c1k', 'c2k', 'c3k', 'slackk',...
    'sqk1e1', 'sqk1e2', 'sqk1e3'...
    'sdqk1e1', 'sdqk1e2', 'sdqk1e3'...
    'u1k1','u2k1',...
    'c1k1', 'c2k1', 'c3k1', 'slackk1',...
    'empty'});
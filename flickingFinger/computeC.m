function C = computeC(state,p)
% give current state q1 q2 q3 dq1 dq3 dq3 and system paramters, calculate
% corilolis matrix
q2 = state(2);
dq1 = state(4);
dq2 = state(5);
C = [ -1/2*p.m2*p.l1*p.l2*sin(q2)*dq2   -1/2*p.m2*p.l1*p.l2*sin(q2)*dq1-1/2*p.m2*p.l1*p.l2*sin(q2)*dq2;
       1/2*p.m2*p.l1*p.l2*sin(q2)*dq1      0 ];
end
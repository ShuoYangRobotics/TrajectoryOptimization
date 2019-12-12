function M = computeM(state,p)
% give current state q1 q2 q3 dq1 dq3 dq3 and system paramters, calculate
% mass matrix
q2 = state(2);
M = [ 1/3*p.m1*p.l1^2 + p.m2*p.l1^2 + 1/3*p.m2*p.l2^2 + p.m2*p.l1*p.l2*cos(q2)     1/3*p.m2*p.l2^2 + 1/2*p.m2*p.l1*p.l2*cos(q2);
                                  1/3*p.m2*p.l2^2 + 1/2*p.m2*p.l1*p.l2*cos(q2)                                   1/3*p.m2*p.l2^2];
end
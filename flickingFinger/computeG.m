function G = computeG(state,p)
% give current state q1 q2 q3 dq1 dq3 dq3 and system paramters, calculate
% gravity matrix
q1 = state(1);
q2 = state(2);
G = [ 1.5*p.m1*p.l1*p.g*sin(q1)+0.5*p.m1*p.g*p.l2*cos(q2)*sin(q1)+0.5*p.m1*p.g*p.l2*cos(q1)*sin(q2);
                                0.5*p.m1*p.g*p.l2*cos(q2)*sin(q1)+0.5*p.m1*p.g*p.l2*cos(q1)*sin(q2)];
end
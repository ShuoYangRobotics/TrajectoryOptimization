function J = computeJs(state,p)
% give current state q1 q2 q3 dq1 dq3 dq3 and system paramters, calculate
% end effector space jacobian
q1 = state(1);
J = [0 -p.l1*cos(q1); 
     0 -p.l1*sin(q1);
     1         1];
end
function J = computeJb(state,p)
% give current state q1 q2 q3 dq1 dq3 dq3 and system paramters, calculate
% end effector body jacobian
q2 = state(2);
J = [p.l2+p.l1*cos(q2) p.l2;
      -p.l2*sin(q2)      0;
               1         1];
end
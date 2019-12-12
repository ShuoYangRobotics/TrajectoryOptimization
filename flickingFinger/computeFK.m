function FK = computeFK(state,p)
% give current state q1 q2 q3 dq1 dq3 dq3 and system paramters, calculate
% end effortor position
q1 = state(1);
q2 = state(2);
FK = [p.l1*sin(q1)+p.l2*sin(q1+q2);-p.l1*cos(q1)-p.l2*cos(q1+q2)];
end
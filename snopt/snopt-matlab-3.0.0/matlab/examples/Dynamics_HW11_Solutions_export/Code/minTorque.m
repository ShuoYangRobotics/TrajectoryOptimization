function cost = minTorque(X)

q = X(:,1:2);
dq = X(:,3:4);
u = X(:,5:6);
q1 = q(:,1);
q2 = q(:,2);
dq1 = dq(:,1);
dq2 = dq(:,2);

dt = 0.02;

% Trapezoidal integration of torque squared 
cost = 0;
for i = 1:length(X)-1
    cost = cost + 0.5*dt*(u(i+1,:)*u(i+1,:)' + u(i,:)*u(i,:)');
end

end
function dz = flickingFingerDynamics(state,u,lambda,p)

% state: angles and angular velocities of robot
% u: control input to finger
% lambda: [lambda_xp,lambda_xm,lambda_z] 
% p: system dynamics
dq_finger = state(4:5);

M = computeM(state,p);
C = computeC(state,p);
G = computeG(state,p);
J = computeJ(state,p);

contactAngle = contactSpeedAngle(state,p);
% rotate lambda force represented in contact frame to world frame
lambda_c = [lambda(1)-lambda(2);lambda(3)];    
lambda_w = [cos(contactAngle)  sin(contactAngle);
            -sin(contactAngle)  cos(contactAngle)]*lambda_c;
ddq_finger = M\(u + J'*lambda_w - C*dq_finger -G);

dq_wheel = state(6);

ddq_wheel = (lambda(1)-lambda(2))/(1/2*p.m3*p.r^2);

dz = [dq_finger;dq_wheel;ddq_finger;ddq_wheel];
end
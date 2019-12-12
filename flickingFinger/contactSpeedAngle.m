function [angle,diffV] = contactSpeedAngle(state,p)
% according to posa paper, we need to calculate relative velocity between
% the wheel and the end effector
% this velocity is along the x axis of a reference frame that sits on the
% contact location, the angle between this frame and world frame is also returned
q1 = state(1);
q2 = state(2);
q3 = state(3);
dq1 = state(4);
dq2 = state(5);
dq3 = state(6);

ee_pos = computeFK(state,p);

% distance vector from wheel center to end effector
normal_vector = [ee_pos(1)-p.xc;
                 ee_pos(2)-p.yc];
unit_normal_vector = normal_vector/norm(normal_vector);
angle = atan2(unit_normal_vector(1), unit_normal_vector(2));
if nargout > 1
Jb = computeJb(state,p);
ee_vel_b = Jb*[dq1;dq2];
ee_vel_w = [cos(q1+q2)  -sin(q1+q2);
            sin(q1+q2)  cos(q1+q2)]*ee_vel_b(1:2);
% 12-11 originally here is rotate counter clock wise, now I think it should
% be clockwisze
% rotate normal vector clockwisely 90degree to get tangent vector
unit_tangent_vector = [0 1; -1 0]*unit_normal_vector ;
% find the component of ee_vel that is perpendicular to normal_vector
% first find the component of ee_vel along the normal vector by project
% ee_vel onto normal_vector
ee_vel_proj = dot(ee_vel_w, unit_normal_vector)*unit_normal_vector;

ee_vel_orth_w = ee_vel_w - ee_vel_proj;

% % get the point that lies on the normal vector and the wheel edge
% edge_point = unit_normal_vector*p.r+[p.xc;p.yc];
% the difference velocity represented in reference frame at contact
% location
ee_vel_orth_ref = [cos(angle)  sin(angle);
                  -sin(angle)  cos(angle)]'*ee_vel_orth_w;
            
wheel_vel = -dq3*p.r;

diffV = ee_vel_orth_ref(1) - wheel_vel;
end
end
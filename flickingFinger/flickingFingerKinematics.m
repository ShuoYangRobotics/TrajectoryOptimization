function [p1,p1e,p2,p2e,circlea,circleb] = flickingFingerKinematics(z,p)
% [p1,p2,dp1,dp2] = acrobotKinematics(z,p)
%
% This function computes the mechanical energy of the acrobot.
%
% INPUTS:
%   z = [4,n] = state vector
%   p = parameter struct:
%       .m1 = elbow mass
%       .m2 = wrist mass
%       .g = gravitational acceleration
%       .l1 = length shoulder to elbow
%       .l2 = length elbow to wrist
%
% OUTPUTS:ls
%   p1 = [2,n] = position of the elbow joint
%   p2 = [2,n] = position of the wrist
%   dp1 = [2,n] = velocity of the elbow joint
%   dp2 = [2,n] = velocity of the wrist
% 
% NOTES:
%   
%   states:
%       1 = q1 = first link angle
%       2 = q2 = second link angle
%       3 = dq1 = first link angular rate
%       4 = dq2 = second link angular rate
%
%   angles: measured from negative j axis with positive convention
%

q1 = z(1,:);
q2 = z(2,:);
q3 = z(3,:);
dq1 = z(4,:);
dq2 = z(5,:);
dq3 = z(6,:);

p1  = [0.5*p.l1*sin(q1);
     -0.5*p.l1*cos(q1)];
p1e = [ p.l1*sin(q1);
       -p.l1*cos(q1)];
p2  = [ p.l1*sin(q1)+0.5*p.l2*sin(q1+q2);
       -p.l1*cos(q1)-0.5*p.l2*cos(q1+q2)];
p2e = [ p.l1*sin(q1)+    p.l2*sin(q1+q2);
       -p.l1*cos(q1)-    p.l2*cos(q1+q2)];
   
circlea = [p.xc+0.7*p.r*cos(q3);
           p.yc+0.7*p.r*sin(q3)];
circleb = [p.xc-0.7*p.r*cos(q3);
           p.yc-0.7*p.r*sin(q3)];
 

end
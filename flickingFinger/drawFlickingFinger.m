function drawFlickingFinger(t,z,p,x0,xF)

clf; hold on;

length = p.l1+p.l2;
axis equal; axis(length*[-1,1,-1.5,0.3]); axis off;

drawCircleRadius = 0.07;
% start
[p1,p1e,p2,p2e,circlea,circleb] = flickingFingerKinematics(x0,p);

pos = [[0;0],p1,p1e,p2,p2e];

plot(0,0,'ks','MarkerSize',10,'LineWidth',4)
plot(pos(1,:),pos(2,:),'Color',[0.8, 0.1, 0.1 0.1],'LineWidth',4)

% end
[p1,p1e,p2,p2e,circlea,circleb] = flickingFingerKinematics(xF,p);

pos = [[0;0],p1,p1e,p2,p2e];

plot(0,0,'ks','MarkerSize',10,'LineWidth',4)
plot(pos(1,:),pos(2,:),'Color',[0.1, 0.1, 0.8 0.1],'LineWidth',4)

% middle
[p1,p1e,p2,p2e,circlea,circleb] = flickingFingerKinematics(z,p);

pos = [[0;0],p1,p1e,p2,p2e];

plot(0,0,'ks','MarkerSize',10,'LineWidth',4)
plot(pos(1,:),pos(2,:),'Color',[0.1, 0.8, 0.1],'LineWidth',4)
pos = [[0;0],p1e,p2e];
plot(pos(1,:),pos(2,:),'k.','MarkerSize',16)

[angle,diffV] = contactSpeedAngle(z(1:6),p);

% draw contacts 6 state 2 control 4 contact 
contacts = [z(9)-z(10);z(11)]*5;
% distance vector from wheel center to end effector
normal_vector = [p2e(1)-p.xc;
                 p2e(2)-p.yc];
unit_normal_vector = normal_vector/norm(normal_vector);
% rotate normal vector counterclockwisely 90degree to get tangent vector
unit_tangent_vector = [0 1; -1 0]*unit_normal_vector ;
force_pos1 = [p2e,p2e+unit_normal_vector*contacts(2)];
plot(force_pos1(1,:),force_pos1(2,:),'Color',[1, 0, 0],'LineWidth',2)
force_pos2 = [p2e,p2e+unit_tangent_vector*contacts(1)];
plot(force_pos2(1,:),force_pos2(2,:),'Color',[1, 0, 0],'LineWidth',2)


pos = [circlea,circleb];
plot(pos(1,:),pos(2,:),'Color',[0, 0, 0],'LineWidth',5);
circle(p.xc,p.yc,p.r,'k');
circle(circlea(1),circlea(2),drawCircleRadius,'g');
circle(circleb(1),circleb(2),drawCircleRadius,'b');


title(sprintf('Flicking Finger Animation,  t = %6.4f', t));

drawnow; pause(0.001); 

end
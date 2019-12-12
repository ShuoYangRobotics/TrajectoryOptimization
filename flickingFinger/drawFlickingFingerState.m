function drawFlickingFingerState(t,x0,xF,p)

figure(4)
clf; hold on;

length = p.l1+p.l2;
axis equal; axis(length*[-1,1,-1.5,0.3]); axis off;

drawCircleRadius = 0.07;
% start
[p1,p1e,p2,p2e,circlea,circleb] = flickingFingerKinematics(x0,p);

pos = [[0;0],p1,p1e,p2,p2e];

plot(0,0,'ks','MarkerSize',10,'LineWidth',4)
plot(pos(1,:),pos(2,:),'Color',[0.1, 0.8, 0.1],'LineWidth',4)
pos = [[0;0],p1e,p2e];
plot(pos(1,:),pos(2,:),'k.','MarkerSize',16)

pos = [circlea,circleb];
plot(pos(1,:),pos(2,:),'Color',[0, 0, 0],'LineWidth',5);
circle(p.xc,p.yc,p.r,'k');
circle(circlea(1),circlea(2),drawCircleRadius,'g');
circle(circleb(1),circleb(2),drawCircleRadius,'b');


title(sprintf('Flicking Finger Start Pose'));

drawnow; 

figure(5)
clf; hold on;

length = p.l1+p.l2;
axis equal; axis(length*[-1,1,-1.5,0.3]); axis off;

drawCircleRadius = 0.07;
% end
[p1,p1e,p2,p2e,circlea,circleb] = flickingFingerKinematics(xF,p);

pos = [[0;0],p1,p1e,p2,p2e];

plot(0,0,'ks','MarkerSize',10,'LineWidth',4)
plot(pos(1,:),pos(2,:),'Color',[0.1, 0.8, 0.1],'LineWidth',4)
pos = [[0;0],p1e,p2e];
plot(pos(1,:),pos(2,:),'k.','MarkerSize',16)

pos = [circlea,circleb];
plot(pos(1,:),pos(2,:),'Color',[0, 0, 0],'LineWidth',5);
circle(p.xc,p.yc,p.r,'k');
circle(circlea(1),circlea(2),drawCircleRadius,'g');
circle(circleb(1),circleb(2),drawCircleRadius,'b');


title(sprintf('Flicking Finger End Pose'));

drawnow; 

end
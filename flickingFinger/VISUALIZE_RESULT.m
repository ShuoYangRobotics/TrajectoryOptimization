
addpath ../../
% must run MAIN First

% load zSoln from a dataset 
load('counterclkwheelrotate_1218_xF_back_as_cst.mat')
% load('counterclkwheelrotate_1216_with_new_obj.mat')
% load('counterclkwheelrotate_1216_with_new_obj.mat')

%% visualize result
% xSoln = reshape(zSoln,nState+nControl+nContactForce,nGrid);
% reshape solution
xSoln = [reshape(zSoln(stateIdx),nState,nGrid);
         reshape(zSoln(controlIdx),nControl,nGrid);
         reshape(zSoln(forceIdx),nContactForce,nGrid)];
% draw using animate tool
A.plotFunc = @(t,z)( drawFlickingFinger(t,z,dyn,x0,xF) );
A.speed = 0.2;
A.figNum = 101;
animate(time(:,1:end),xSoln(:,1:end),A)

% plot trajectories
state_soln = reshape(zSoln(stateIdx),nState,nGrid);
ctrl_soln = reshape(zSoln(controlIdx),nControl,nGrid);
contact_soln = reshape(zSoln(forceIdx),nContactForce,nGrid);

%% solution ceq and c
dim_boundary_cst = 9;
dim_ceq = 6;
dim_c = 6;

% ceq = zeros(6+(nGrid-1)*dim_ceq,1);
ceq = zeros(dim_boundary_cst+(nGrid-1)*dim_ceq,1);
c   = zeros((nGrid-1)*dim_c,1);

gceq = zeros(nZ,dim_boundary_cst+(nGrid-1)*dim_ceq);
gc = zeros(nZ,(nGrid-1)*dim_c);


ceq(1:6) = (state_soln(:,1)-x0).^2;
ceq(7:9) = (state_soln(1:3,end)-xF(1:3)).^2;
gceq(1:6,1:6) = diag((2*(state_soln(:,1)-x0)));
gceq(nState*(nGrid-1)+1:nState*(nGrid-1)+3,7:9) = diag(2*(state_soln(1:3,end)-xF(1:3)));


idxUpp = nGrid
idxCur = idxUpp - 1
idxLow = idxUpp - 2

qk1    = state_soln(1:3,idxLow)
dqk1   = state_soln(4:6,idxLow)

qk2  = state_soln(1:3,idxCur)
dqk2 = state_soln(4:6,idxCur) 

qk3  = state_soln(1:3,idxUpp)
dqk3 = state_soln(4:6,idxUpp)

state_cst1  = [qk2;dqk2;qk1;dqk1;ctrl_soln(:,idxCur);contact_soln(:,idxLow);contact_soln(:,idxCur)];
state_cst2  = [qk3;dqk3;qk2;dqk2;ctrl_soln(:,idxUpp);contact_soln(:,idxCur);contact_soln(:,idxUpp)];
  

% p stands for "part"
[pc1,~,pcz1,pczi1,~] = autoGen_c_grad(dt,state_cst1(1),state_cst1(2),state_cst1(3),...
                                   state_cst1(4),state_cst1(5),state_cst1(6),...
                                   state_cst1(7),state_cst1(8),state_cst1(9),...
                                   state_cst1(10),state_cst1(11),state_cst1(12),...
                                   state_cst1(13),state_cst1(14),...
                                   state_cst1(15),state_cst1(16),state_cst1(17),state_cst1(18),...
                                   state_cst1(19),state_cst1(20),state_cst1(21),state_cst1(22),...
                                   0);

[pceq1,~,pceqz1,pceqzi1,~] = autoGen_ceq_grad(dt,state_cst1(1),state_cst1(2),state_cst1(3),...
                                           state_cst1(4),state_cst1(5),state_cst1(6),...
                                           state_cst1(7),state_cst1(8),state_cst1(9),...
                                           state_cst1(10),state_cst1(11),state_cst1(12),...
                                           state_cst1(13),state_cst1(14),...
                                           state_cst1(15),state_cst1(16),state_cst1(17),state_cst1(18),...
                                           state_cst1(19),state_cst1(20),state_cst1(21),state_cst1(22),...
                                           0);
                                       
[pc2,~,pcz2,pczi2,~] = autoGen_c_grad(dt,state_cst2(1),state_cst2(2),state_cst2(3),...
                                   state_cst2(4),state_cst2(5),state_cst2(6),...
                                   state_cst2(7),state_cst2(8),state_cst2(9),...
                                   state_cst2(10),state_cst2(11),state_cst2(12),...
                                   state_cst2(13),state_cst2(14),...
                                   state_cst2(15),state_cst2(16),state_cst2(17),state_cst2(18),...
                                   state_cst2(19),state_cst2(20),state_cst2(21),state_cst2(22),...
                                   0);

[pceq2,~,pceqz2,pceqzi2,~] = autoGen_ceq_grad(dt,state_cst2(1),state_cst2(2),state_cst2(3),...
                                   state_cst2(4),state_cst2(5),state_cst2(6),...
                                   state_cst2(7),state_cst2(8),state_cst2(9),...
                                   state_cst2(10),state_cst2(11),state_cst2(12),...
                                   state_cst2(13),state_cst2(14),...
                                   state_cst2(15),state_cst2(16),state_cst2(17),state_cst2(18),...
                                   state_cst2(19),state_cst2(20),state_cst2(21),state_cst2(22),...
                                   0);                                      
%     size(pc)
  
                                        
% ceq(6+(idxUpp-2)*6+1:6+(idxUpp-2)*6+6) = pceq1;  
% c((idxUpp-2)*12+1:(idxUpp-2)*12+12) = pc1;

% assemble idices 
% idx = [nState*(idxLow-1)+1:nState*idxUpp  nState*nGrid+nControl*(idxLow-1)+1:nState*nGrid+nControl*idxLow nState*nGrid+nControl*nGrid+nContactForce*(idxLow-1)+1:nState*nGrid+nControl*nGrid+nContactForce*idxUpp];

k2 = zeros(12,22);
k2(pczi1) = real(pcz1);
% gc(idx,(idxUpp-2)*12+1:(idxUpp-2)*12+12) = k2';

k = zeros(6,22);
k(pceqzi1) = real(pceqz1);
% gceq(idx,6+(idxUpp-2)*6+1:6+(idxUpp-2)*6+6) = k';

pceq2 - (pceq1+k*(state_cst2-state_cst1))
pc2 - (pc1+k2*(state_cst2-state_cst1))
%%
% run system dynamics using control again
% x0 
% xF
figure('Name','Visualize Data','WindowState', 'maximized')
subplot(3,3,1);
plot(1:nGrid,state_soln(1,:)); title('state q1') 
subplot(3,3,2);
plot(1:nGrid,state_soln(2,:)); title('state q2') 
subplot(3,3,3);
plot(1:nGrid,state_soln(3,:)); title('state q3') 
subplot(3,3,4);
plot(1:nGrid,ctrl_soln(1,:)); title('control u1') 
subplot(3,3,5);
plot(1:nGrid,ctrl_soln(2,:)); title('control u2') 
subplot(3,3,6);
plot(1:nGrid,contact_soln(3,:)); title('contact normal') 
subplot(3,3,7);
plot(1:nGrid,contact_soln(1,:)-contact_soln(2,:)); title('contact tangent') 
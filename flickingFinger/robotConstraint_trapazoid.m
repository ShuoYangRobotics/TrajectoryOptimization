function [c, ceq, gc, gceq] = robotConstraint_trapazoid(dt,z,x0,xF,nGrid,nState,nControl,nContactForce,p)

nZ = nState*nGrid + nControl*nGrid + nContactForce*nGrid;
stateIdx = 1:nState*nGrid;
controlIdx = nState*nGrid+1:nState*nGrid + nControl*nGrid;
forceIdx = nState*nGrid+nControl*nGrid+1:nZ;

x = reshape(z(stateIdx),nState,nGrid); 
u = reshape(z(controlIdx),nControl,nGrid);
contact = reshape(z(forceIdx),nContactForce,nGrid);

% lambda = [contact(1,:)-contact(2,:);contact(3,:)];
% slack = contact(4,:);

% to speed up calculate, preallocate memory space for c and ceq
% follow Posa paper Equation 8-16
% ceq dim = 12 (initial and final boundary) +
%           (nGrid-1)*6 (collocation constraint) +
%           (nGrid-1)*4 (complementarity constraint eqn 13-16)
% c   dim = 
%           (nGrid-1)*1 (complementarity eqn 8) +
%           (nGrid-1)*4 (complementarity eqn 9) +
%           (nGrid-1)*1 (complementarity eqn 10)+
%           (nGrid-1)*1 (complementarity eqn 11)+
%           (nGrid-1)*1 (complementarity eqn 12)

ceq = zeros(12+(nGrid-1)*10,1);
c   = zeros((nGrid-1)*8,1);
if nargout > 2
    gceq = zeros(nZ,12+(nGrid-1)*10);
    gc = zeros(nZ,(nGrid-1)*8);
end

ceq(1:6) = (x(:,1)-x0).^2;
ceq(7:12) = (x(:,end)-xF).^2;
if nargout > 2
    gceq(1:6,1:6) = diag((2*(x(:,1)-x0)));
    gceq(nState*(nGrid-1)+1:nState*nGrid,7:12) = diag(2*(x(:,end)-xF));
end

for idxUpp = 2:nGrid
    idxLow = idxUpp - 1;
    qk    = x(1:3,idxLow);
    dqk   = x(4:6,idxLow);
    
    qk1  = x(1:3,idxUpp);
    dqk1 = x(4:6,idxUpp);  %[qk1;dqk1] [qk;dqk]

%     M = computeM([qk1;dqk1],p);
%     C = computeC([qk1;dqk1],p);
%     G = computeG([qk1;dqk1],p);
%     Jb = computeJb([qk1;dqk1],p);
% 
%     contactAnglek1 = contactSpeedAngle([qk1;dqk1],p);
%     [contactAnglek,diffV] = contactSpeedAngle([qk;dqk],p);
%     % rotate lambda force represented in contact frame to world frame
%     lambda_ck1 = [lambda(1,idxUpp);lambda(2,idxUpp)]; 
%     lambda_wk1 = [cos(contactAnglek1)  sin(contactAnglek1);
%                  -sin(contactAnglek1)  cos(contactAnglek1)]*lambda_ck1;
% 
%     lambda_tk1 = [[cos(qk1(1)+qk1(2))  sin(qk1(1)+qk1(2));
%                   -sin(qk1(1)+qk1(2))   cos(qk1(1)+qk1(2))]*lambda_wk1;0];
% 
%     phi = wheelConstraint([qk;dqk],p);
    
    % p stands for "part"
    [pc,~,pcz,pczi,~] = autoGen_c_grad(qk(1),qk(2),qk(3),...
                                            dqk(1),dqk(2),dqk(3),...
                                            qk1(1),qk1(2),qk1(3),...
                                            dqk1(1),dqk1(2),dqk1(3),...
                                            u(1,idxUpp),u(2,idxUpp),...
                                            contact(1,idxLow),contact(2,idxLow),contact(3,idxLow),contact(4,idxLow),...
                                            contact(1,idxUpp),contact(2,idxUpp),contact(3,idxUpp),contact(4,idxUpp),0);
    
    [pceq,~,pceqz,pceqzi,~] = autoGen_ceq_grad(qk(1),qk(2),qk(3),...
                                            dqk(1),dqk(2),dqk(3),...
                                            qk1(1),qk1(2),qk1(3),...
                                            dqk1(1),dqk1(2),dqk1(3),...
                                            u(1,idxUpp),u(2,idxUpp),...
                                            contact(1,idxLow),contact(2,idxLow),contact(3,idxLow),contact(4,idxLow),...
                                            contact(1,idxUpp),contact(2,idxUpp),contact(3,idxUpp),contact(4,idxUpp),0);
%     size(pc)
    
                                        
    ceq(12+(idxUpp-2)*10+1:12+(idxUpp-2)*10+10) = pceq;  
    c((idxUpp-2)*8+1:(idxUpp-2)*8+8) = pc;
    
    if nargout > 2
        % assemble idices 
        idx = [nState*(idxLow-1)+1:nState*idxUpp  nState*nGrid+nControl*(idxLow-1)+1:nState*nGrid+nControl*idxLow nState*nGrid+nControl*nGrid+nContactForce*(idxLow-1)+1:nState*nGrid+nControl*nGrid+nContactForce*idxUpp];
        
        k2 = zeros(8,22);
        k2(pczi) = real(pcz);
        gc(idx,(idxUpp-2)*8+1:(idxUpp-2)*8+8) = k2';
               
        k = zeros(10,22);
        k(pceqzi) = real(pceqz);
        gceq(idx,12+(idxUpp-2)*10+1:12+(idxUpp-2)*10+10) = k';
    end


%     ceq(12+(idxUpp-2)*10+1:12+(idxUpp-2)*10+10) = [qk-qk1+dt*dqk1;  %dim3
%                              M*(dqk1(1:2)-dqk(1:2))+dt*(C*dqk1(1:2)+G-u(:,idxUpp)+Jb'*(lambda_tk1)); %dim2
%                              (1/2*p.m3*p.r^2)*(dqk1(3)-dqk(3))+dt*(lambda_ck1(1)) %dim1
%                              phi*contact(3,idxLow);
%                              (p.miu*contact(3,idxLow)-contact(1,idxLow)-contact(2,idxLow))*slack(idxLow);
%                              (slack(idxLow)+diffV)*contact(1,idxLow);
%                              (slack(idxLow)-diffV)*contact(2,idxLow)];  
%     c((idxUpp-2)*8+1:(idxUpp-2)*8+8) = [-phi;  %dim1
%                                         -contact(:,idxLow); %dim4
%                                         -(p.miu*contact(3,idxLow)-contact(1,idxLow)-contact(2,idxLow)); %dim1
%                                         -(slack(idxLow)+diffV); %dim1
%                                         -(slack(idxLow)-diffV)]; %dim1
end

end
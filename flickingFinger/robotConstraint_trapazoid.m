function [c, ceq, gc, gceq] = robotConstraint_trapazoid(dt,z,x0,xF,nGrid,nState,nControl,nContactForce,p)

nZ = nState*nGrid + nControl*nGrid + nContactForce*nGrid;
stateIdx = 1:nState*nGrid;
controlIdx = nState*nGrid+1:nState*nGrid + nControl*nGrid;
forceIdx = nState*nGrid+nControl*nGrid+1:nZ;

x = reshape(z(stateIdx),nState,nGrid); 
u = reshape(z(controlIdx),nControl,nGrid);
contact = reshape(z(forceIdx),nContactForce,nGrid);

ceq = zeros(12+(nGrid-1)*6,1);
c   = zeros((nGrid-1)*12,1);
if nargout > 2
    gceq = zeros(nZ,12+(nGrid-1)*6);
    gc = zeros(nZ,(nGrid-1)*12);
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
    dqk1 = x(4:6,idxUpp);  

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
    
                                        
    ceq(12+(idxUpp-2)*6+1:12+(idxUpp-2)*6+6) = pceq;  
    c((idxUpp-2)*12+1:(idxUpp-2)*12+12) = pc;
    
    if nargout > 2
        % assemble idices 
        idx = [nState*(idxLow-1)+1:nState*idxUpp  nState*nGrid+nControl*(idxLow-1)+1:nState*nGrid+nControl*idxLow nState*nGrid+nControl*nGrid+nContactForce*(idxLow-1)+1:nState*nGrid+nControl*nGrid+nContactForce*idxUpp];
        
        k2 = zeros(12,22);
        k2(pczi) = real(pcz);
        gc(idx,(idxUpp-2)*12+1:(idxUpp-2)*12+12) = k2';
               
        k = zeros(6,22);
        k(pceqzi) = real(pceqz);
        gceq(idx,12+(idxUpp-2)*6+1:12+(idxUpp-2)*6+6) = k';
    end
end

end
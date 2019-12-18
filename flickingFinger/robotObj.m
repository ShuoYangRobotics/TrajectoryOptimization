function [J,GJ] = robotObj(dt,z,x0,xF,nGrid,nState,nControl,nContactForce)

nZ = nState*nGrid + nControl*nGrid + nContactForce*nGrid;
stateIdx = 1:nState*nGrid;
controlIdx = nState*nGrid+1:nState*nGrid + nControl*nGrid;
forceIdx = nState*nGrid+nControl*nGrid+1:nZ;

x = reshape(z(stateIdx),nState,nGrid); 
u = reshape(z(controlIdx),nControl,nGrid);
contact = reshape(z(forceIdx),nContactForce,nGrid);

% minimize final state difference
state_diff = 600*(x(:,end)-xF);

J = (state_diff'*state_diff);
GJ = zeros(1,nZ);
GJ(nState*nGrid-5:nState*nGrid) = 2*600^2*(x(:,end)-xF);


for idxLow = 1:(nGrid-1)
    idxUpp = idxLow+1;

    [jeq, ~, jeqz, jeqzi, ~] = autoGen_J_grad(dt, x(1,idxLow),x(2,idxLow),x(3,idxLow),...
                                                  x(4,idxLow),x(5,idxLow),x(6,idxLow),...
                                                  u(1,idxLow),u(2,idxLow),...
                                                  contact(1,idxLow),contact(2,idxLow),contact(3,idxLow),contact(4,idxLow),...
                                                  x(1,idxUpp),x(2,idxUpp),x(3,idxUpp),...
                                                  x(4,idxUpp),x(5,idxUpp),x(6,idxUpp),...
                                                  u(1,idxUpp),u(2,idxUpp),...
                                                  contact(1,idxUpp),contact(2,idxUpp),contact(3,idxUpp),contact(4,idxUpp),...
                                                  0);
    J = J + jeq;
    
    k = zeros(1,24);
    k(jeqzi) = real(jeqz);
    idx = [nState*(idxLow-1)+1:nState*idxUpp  nState*nGrid+nControl*(idxLow-1)+1:nState*nGrid+nControl*idxUpp nState*nGrid+nControl*nGrid+nContactForce*(idxLow-1)+1:nState*nGrid+nControl*nGrid+nContactForce*idxUpp];
        
    GJ(idx) = GJ(idx) + k;
end
GJ = GJ';
end
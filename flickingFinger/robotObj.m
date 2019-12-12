function J = robotObj(dt,z,x0,xF,nGrid,nState,nControl,nContactForce)

nZ = nState*nGrid + nControl*nGrid + nContactForce*nGrid;
stateIdx = 1:nState*nGrid;
controlIdx = nState*nGrid+1:nState*nGrid + nControl*nGrid;
forceIdx = nState*nGrid+nControl*nGrid+1:nZ;

x = reshape(z(stateIdx),nState,nGrid); 
u = reshape(z(controlIdx),nControl,nGrid);
contact = reshape(z(forceIdx),nContactForce,nGrid);
force_X = contact(1,:)-contact(2,:);
force_Z = contact(3,:);

idxLow = 1:(nGrid-1);
idxUpp = 2:nGrid;
uappx = (0.5*dt*(u(:,idxUpp)+u(:,idxLow)));
uappx = uappx(:);
% minimize control,  minimize slack variable
% the control goal is what?
J = uappx'*uappx + force_X*force_X' + force_Z*force_Z';
end
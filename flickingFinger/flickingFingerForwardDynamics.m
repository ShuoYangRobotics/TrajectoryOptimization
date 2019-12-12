function u = flickingFingerForwardDynamics(state,p)



C = computeC(state,p);
G = computeG(state,p);

u = C*state(2:3) + G;
end
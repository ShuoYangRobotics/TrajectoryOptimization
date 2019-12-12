function phi = wheelConstraint(state,p)

    
    ee_pos = computeFK(state,p);
    phi = (ee_pos(1)-p.xc)^2+(ee_pos(2)-p.yc)^2-p.r^2;
end
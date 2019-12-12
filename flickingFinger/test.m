n_x = -1
n_y = 2

angle = atan2(n_x,n_y)

mtx = [cos(angle) sin(angle);
      -sin(angle) cos(angle)];

vec_c = [1;0];

vec_s = mtx*vec_c
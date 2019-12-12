function [m, mi, mz, mzi, dim] = computeGradients(M,z,empty)
%
% This function computes the gradients of a matrix M with respect the the
% variables in z, and then returns both the matrix and its gradient as
% column vectors of their non-zero elements, along with the linear indicies
% to unpack them. It also simplifies m and mz.
%
% INPUTS:
%   M = [na, nb] = symbolic matrix
%   z = [nc, 1] = symbolic vector
%
% OUTPUTS:
%   m = [nd, 1] = symbolic vector of non-zero elements in M
%   i = [nd, 1] = linear indicies to map m --> [na,nb] matrix
%   mz = [ne, 1] = symbolic vector of non-zero elements in Mz
%   iz = [ne, 1] = linear indicies to map mz --> [na,nb,nc] array
%   dim = [3,1] = [na,nb,nc] = dimensions of 3d version of mz
%

[na, nb] = size(M)
nc = size(z,1)
M = simplify(M);

mz2 = jacobian(M(:),z);  %Compute jacobian of M, by first reshaping M to be a column vector
mz3 = reshape(mz2,na,nb,nc); %Expand back out to a three-dimensional array
mz3 = simplify(mz3);

% Extract non-zero elements to a column vector:
mi = find(M);
m = M(mi);
mzi = find(mz3);
mz = mz3(mzi); mz = mz(:);  %Collapse to a column vector
dim = [na,nb,nc];

% Pad any constant terms with "empty" to permit vectorization:
m = vectorizeHack(m, z, empty);
mz = vectorizeHack(mz, z, empty);

end
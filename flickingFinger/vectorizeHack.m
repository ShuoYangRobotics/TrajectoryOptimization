function x = vectorizeHack(x, z, empty)
%
% This function searches for any elements of x that are not dependent on
% any element of z. In this case, the automatically generated code will
% fail to vectorize properly. One solution is to add an array of zeros
% (empty) to the element.
%
% x = column vector of symbolic expressions
% z = column vector of symbolic variables
% z = symbolic variable, which the user will set equal to zero.
%

% Compute dependencies
g = jacobian(x,z);

% Check for rows of x with no dependence on z
[n,m] = size(g);
idxConst = true(n,1);
for i=1:n
    for j=1:m
        if ~isequal(sym(0),g(i,j))
            idxConst(i) = false;
            break;
        end
    end
end

% Add empty to those enteries
x(idxConst) = x(idxConst) + empty;

end
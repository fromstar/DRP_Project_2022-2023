function [c ceq] = nlinconst(x)

c(1) = -abs(x(4)) + 0.1;
c(2) = -abs(x(5)) + 0.1;

ceq = [];
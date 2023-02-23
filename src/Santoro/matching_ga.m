function [x,f,fval] = matching_ga(M0, M1, id)

% Choose a point which will be the center of rotation
x_centerM0 = M0(1, id);
y_centerM0 = M0(2, id);

x_centerM1 = M1(1, id);
y_centerM1 = M1(2, id);

% Create a matrix which will be used later in calculations
centerM0 = repmat([x_centerM0; y_centerM0], 1, size(M0,2));
centerM1 = repmat([x_centerM1; y_centerM1], 1, size(M1,2));

% shift points in the plane so that the center of rotation is at the origin
s0 = M0 - centerM0;  
s1 = M1 - centerM1;     

% calculate H
H = s0 * s1';
[U,S,V] = svd(H);
R = V*U';

if det(R) < 0
    disp("det(R) < 0, reflection detected!, correcting for it ...");
    f = -1;
else
    f = 1;
end

% objfcn = @(input) [cos(deg2rad(input(1))), -sin(deg2rad(input(1))); sin(deg2rad(input(1))), cos(deg2rad(input(1)))] * [1, 0; 0, f] * (s1(:,2:end)) + [input(2); input(3)];
objfcn = @(input) [cos(deg2rad(input(1))), -sin(deg2rad(input(1))); sin(deg2rad(input(1))), cos(deg2rad(input(1)))] * [input(4), 0; 0, input(5)] * (s1(:,2:end)) + [input(2); input(3)];

fitfcn = @(input) sum(sqrt(sum((s0(:,2:end) - objfcn(input)).^2,1)),2);

% finds a local minimum x to fitnessfcn, subjectto the linear inequalities 
A = [];
b = [];

LB = [0 -200 -200 -1 -1];
UB = [360 200 200 1 1];

% Find the solution
opts = optimoptions('ga','MaxStallGenerations',100,'FunctionTolerance',5e-3,'MaxGenerations',300,'PlotFcn',@gaplotbestfun);
opts = optimoptions(opts,'Display','off','PopulationSize',1000);

[x,fval] = ga(fitfcn,5,A,b,[],[],LB,UB,@nlinconst,[4,5],opts);








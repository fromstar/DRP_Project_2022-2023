function Sfinal = maccgen(Dt)
%% First Step
close all
clc

n_agent = size(Dt,1);
e = ones(n_agent,1);
I = eye(n_agent);

H = I - (e * transpose(e))/n_agent;

UAU = -0.5 * H * Dt * H;

[U,T] = eig(UAU);
t = sort(diag(T),'descend');
Tnew = zeros(n_agent,n_agent);

for i = 1:n_agent
    Tnew(i,i) = t(i);
end

S = sqrt(T) * transpose(U);

fake_zero = 0.0001;


for i = 1:(n_agent)
    if (abs(S(i,1)) >= fake_zero)
        C(i,:) = S(i,:) ;
    end
end
S = C;
S(S(:,1)==0,:) = [];

Slol = S;
%% Fix a coord system
s1 = S(1,1);
s2 = S(2,1);
St1 = ones(1,n_agent);
St2 = ones(1,n_agent);
St1 = St1 * s1;
St2 = St2 * s2;
St = [St1;St2];
S = S - St;
Slol1 = Slol - St;

%% Second step
dS1 = zeros(2,n_agent);
dx2 = 1.9;
dy2 = 0;
dS1(:,1) = [dx2,dy2];
% S1 = A1 + dS1;
syms st ct sdt cdt sett cet
Dt2 = zeros(n_agent);
dDt2 = zeros(n_agent);

for i = 1:n_agent
    for j= 1:n_agent
        dDt2(i,j) = (norm(dS1(:,i) - dS1(:,j)))^2;
    end
end

a = zeros(1,n_agent - 1);
b = zeros(1,n_agent - 1);
c = zeros(1,n_agent - 1);

for i=1:(n_agent -1)
    a(i) = Dt(1,i+1) - Dt2(1,i+1) + dx2^2 + dy2^2;
    b(i) = -2 * (Slol1(1,i+1)*dx2 + Slol1(2,i+1)*dy2);
    c(i) = 2 * (Slol1(1,i+1)*dy2 - Slol1(2,i+1)*dx2);
end

eq1  = a(1) + b(1)*ct + c(1)*st == 0;
eq2  = a(2) + b(2)*ct + c(2)*st == 0;
eq3  = a(3) + b(3)*ct + c(3)*st == 0;

eqns = [eq1,eq2];
vars = [st,ct];
sol = solve(eqns,vars);

theta = atan2(sol.st , sol.ct);

M = [[cos(theta),sin(theta)];[-sin(theta),cos(theta)]];
Ssemi1 = M * Slol1;

%% point three

dS2 = zeros(2,n_agent);
dx3 = 0;
dy3 = 1.5;
dS2(:,1) = [dx3,dy3];
% S2 = A1 + dS2;
Dt3 = zeros(n_agent);
dDt3 = zeros(n_agent);

for i = 1:n_agent
    for j= 1:n_agent
        dDt3(i,j) = (norm(dS2(:,i) - dS2(:,j)))^2;
    end
end

ad = zeros(1,n_agent - 1);
bd = zeros(1,n_agent - 1);
cd = zeros(1,n_agent - 1);

for i=1:(n_agent - 1)
    ad(i) = Dt(1,i+1) - Dt3(1,i+1) + dx3^2 + dy3^2;
    bd(i) = -2*(Ssemi1(1,i+1)*dx3 + Ssemi1(2,i+1)*dy3);
    cd(i) = 2*(Ssemi1(1,i+1)*dy3 - Ssemi1(2,i+1)*dx3);
end


eq1d  = ad(1) + bd(1)*cdt + cd(1)*sdt == 0;
eq2d  = ad(2) + bd(2)*cdt + c(2)*sdt == 0;
eq3d  = ad(3) + bd(3)*cdt + c(3)*sdt == 0;

eqnsd = [eq1d,eq2d];
varsd = [sdt,cdt];

sold = solve(eqnsd,varsd);

thetad = atan2(sold.sdt , sold.cdt);


if (abs(thetad) <= fake_zero)
    Sfinal = M * Slol1;
else
    Spart1 = [[-1,0];[0,1]] * Slol1;
    ae = zeros(1,3);
    be = zeros(1,3);
    ce = zeros(1,3);

    for i=1:(n_agent - 1)
        ae(i) = Dt(1,i+1) - Dt2(1,i+1) + dx2^2 + dy2^2;
        be(i) = -2 * (Spart1(1,i+1)*dx2 + Spart1(2,i+1)*dy2);
        ce(i) = 2 * (Spart1(1,i+1)*dy2 - Spart1(2,i+1)*dx2);
    end



    eq1e  = ae(1) + be(1)*cet + ce(1)*sett == 0;
    eq2e  = ae(2) + be(2)*cet + ce(2)*sett == 0;
    
    eqnse = [eq1e,eq2e];
    varse = [sett,cet];

    sole = solve(eqnse,varse);

    thetae = atan2(sole.sett , sole.cet);

    Sfinal = [[cos(thetae),sin(thetae)];[-sin(thetae),cos(thetae)]] * [[-1,0];[0,1]] * Slol1;
end

figure
% scatter(A1(1,:),A1(2,:),'o')
hold on
scatter(Sfinal(1,:),Sfinal(2,:),'x')
title({'Real value vs estimated';'solved'})
end
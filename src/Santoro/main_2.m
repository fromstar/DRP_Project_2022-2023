function [cmds, movement] = main_2(Reference, Theta, sz)

% Set environment interpreter
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');

% Noise distance measurement
mu_r = 0.0;
sigma_r = 0.15;

% Problem size
dim = 2; % 2D-space
% sz = 4; % number of agents

% Generate random agents
% Set velocity (m/s)
v = 1;
% Reference = rand(dim,sz).*15;  % Initial true configuration
% Theta = (rand(sz,1)-0.5)*360; % Assign random orientation to each agent
% Theta(1) = 0; % Set the first one, only for ease the visualization

% Reference = [4	20	4 22;	20	20	4 4];
% Theta = [0 0 0 90];

% Estimate initial configuration via MDS algorithm
D = get_distances(Reference, mu_r, sigma_r);
[P0, D0_new] = s_mds(D, sz, dim);
P0 = P0 - P0(:,1);
P0 = [-1, 0; 0, 1] * P0;

% Create first agent properties
[x1,y1] = agent(Reference(:,1), Theta(1), v, 0);

% Set simulation
figure; hold on; axis equal
xlim([-25 25])
ylim([-25 25])

h = scatter(Reference(1,1),Reference(2,1),80,'red');
h.XDataSource = 'x1';
h.YDataSource = 'y1';

str = string(1:size(Reference,2));
% Reference
scatter(Reference(1,:),Reference(2,:),120,'filled');
text(Reference(1,:)+0.05, Reference(2,:)+0.05, str);
% First estimation
scatter(P0(1,:),P0(2,:),120);
text(P0(1,:)+0.05, P0(2,:)+0.05, str);

legend("Moving node", "Reference", "Initial estimation")
set(gca,'FontSize',24)

trigger = 0;
theta = Theta(1);
step = 1;
fval_thresh = sigma_r*sz;
theta_step = 90;
for i = 2:step:10
    % Let node 1 move and turn left after certain time
    if i > 5 && trigger == 0
        theta = Theta(1) + theta_step;
        trigger = 1;
        
        % Estimate second configuration via MDS algorithm
        D = get_distances([[x1(end);y1(end)], Reference(:, 2:end)], mu_r, sigma_r);
        [P1, D1] = s_mds(D,sz,dim);
        pause(0.5)
        
    end

    [x1(i),y1(i)] = agent([x1(end),y1(end)], theta, v, step);
%     refreshdata
    drawnow
end

% Estimate third configuration via MDS algorithm
D = get_distances([[x1(end);y1(end)], Reference(:, 2:end)], mu_r, sigma_r);
[P2, D2] = s_mds(D,sz,dim);

%% Compute correction step 1
% ID of moving node
id = 1;

% Center the map on moving node
P1 = P1 - P1(:,1);

% Find the solution
fval = 10;
while fval>fval_thresh
    [th,~,fval]  = matching_ga(P0, P1, id);
end

% Reflection matrix
R = [cos(deg2rad(th(1))), -sin(deg2rad(th(1))); sin(deg2rad(th(1))), cos(deg2rad(th(1)))];
R1 = R;
R_alpha = [th(4), 0; 0, th(5)];

% P1 rotated and translated to match P0_new
P1_rot = R * R_alpha * P1(:,2:end) + [th(2); th(3)];   

% Store the displacement of the moving node
shift = (P0(:,1) + [th(2); th(3)]);
displacement = [P0(:,1), shift];

% Add moving node to the map
P1_rot_mn = [displacement(:,end), P1_rot];

P1_rot_c = P1_rot_mn - P1_rot_mn(:,1);

%% Compute correction step 2
% Center the map on moving node
P2 = P2 - P2(:,1);

% Find the solution
fval = 1;
while fval>fval_thresh
    [th,~,fval]  = matching_ga(P1_rot_c, P2, id);
end
% Reflection matrix
R = [cos(deg2rad(th(1))), -sin(deg2rad(th(1))); sin(deg2rad(th(1))), cos(deg2rad(th(1)))];
R_alpha = [th(4), 0; 0, th(5)];

% P2 rotated and translated to match P1_rot
P2_rot = R * R_alpha * P2(:,2:end) + [th(2); th(3)];   

% Store the displacement of the moving node
shift = (displacement(:,end) + [th(2); th(3)]);
displacement = [displacement, shift];

% Add moving node to the map
P2_rot_mn = [displacement(:,end), P2_rot];
P2_rot_c = P2_rot_mn - P2_rot_mn(:,1);

P2_rot_c_P1 = P2_rot + P1_rot_mn(:,1);

%% Final correction
D = get_distances([displacement(:,2), P2_rot_c_P1],mu_r,sigma_r);
[~, D1_c] = s_mds(D,sz,dim);

D = get_distances([displacement(:,3), P2_rot_c_P1],mu_r,sigma_r);
[~, D2_c] = s_mds(D,sz,dim);

% Mirroring x-axis
P2_rot_c_f = [1, 0; 0, -1] * P2_rot_c_P1;
P2_rot_c_f = [displacement(:,3), P2_rot_c_f];

D = get_distances(P2_rot_c_f,mu_r,sigma_r);
[~, D2_c_f] = s_mds(D,sz,dim);

% P2_rot_c_f = [1, 0; 0, -1] * [displacement(:,3), P2_rot_c_P1];
% [~, D2_c_f] = mds(P2_rot_c_f,mu_r,sigma_r);


% Mirroring y-axis
P2_rot_c_f_2 = [-1, 0; 0, 1] * P2_rot_c_P1;
P2_rot_c_f_2 = [displacement(:,3), P2_rot_c_f_2];
D = get_distances(P2_rot_c_f_2,mu_r,sigma_r);
[~, D2_c_f_2] = s_mds(D,sz,dim);

% Mirroring xy-axis
P2_rot_c_f_3 = [-1, 0; 0, -1] * P2_rot_c_P1;
P2_rot_c_f_3 = [displacement(:,3), P2_rot_c_f_3];
D = get_distances(P2_rot_c_f_3,mu_r,sigma_r);
[~, D2_c_f_3] = s_mds(D,sz,dim);

% P2_rot_c_f_2 = [-1, 0; 0, 1] * [displacement(:,3), P2_rot_c_P1];
% [~, D2_c_f_2] = mds(P2_rot_c_f_2,mu_r,sigma_r);


difference_true = (D2 - D1);
difference_1 = D2_c - D1;
difference_2 = D2_c_f - D1;
difference_2_2 = D2_c_f_2 - D1;
difference_2_3 = D2_c_f_3 - D1;
differences = [abs(sum(difference_1 - difference_true,"all")), abs(sum(difference_2 - difference_true,"all")), abs(sum(difference_2_2 - difference_true,"all")), abs(sum(difference_2_3 - difference_true,"all"))];
[value,index] = min(differences);

if index == 1
    disp("No reflection detected");
    map = P2_rot_c_P1;

elseif index == 2
    disp("Reflection around x-axis detected");
    R_alpha = [1, 0; 0, -1];
    map = R_alpha * P2_rot_c_P1;

elseif index == 3
    disp("Reflection around y-axis detected")
    R_alpha = [-1, 0; 0, 1];
    map = R_alpha * P2_rot_c_P1;

elseif index == 4
    disp("Reflection around y-axis detected")
    R_alpha = [-1, 0; 0, -1];
    map = R_alpha * P2_rot_c_P1;
end

% Compute the cross product to determine the orientation sign
rotation = cross([displacement(:,2);0],[displacement(:,3);0]);

if  theta_step > 0 && rotation(3) < 0 || theta_step < 0 && rotation(3) > 0
    disp("Mirroring!")
    R_alpha = [-1, 0; 0, 1];
    map = R_alpha * P2_rot_c_P1;
    displacement = R_alpha * displacement;
end


%% Plot
% ----------------- PLOT step 1 -------------------------------------------
figure; hold on; axis equal

% Label for node
str = string(1:size(Reference,2));

% Plot initial configuration
scatter(P0(1,:),P0(2,:),80)
text(P0(1,:)+0.05, P0(2,:)+0.05, str);

% Plot second configuration alligned with the initial configuration
scatter(P1_rot(1,:),P1_rot(2,:),400,'s')
text(P1_rot(1,:)+0.05, P1_rot(2,:)+0.05, str(:,2:end));

% Plot displacement of moving node
scatter(displacement(1,2),displacement(2,2),80,'red')
plot(displacement(1,1:2),displacement(2,1:2),'r--')

legend("$P^0_{new}$","$P^1_{rot}$","Moving node", "Trajectory")
set(gca,'FontSize',30)

% ----------------- PLOT step 2 -------------------------------------------
figure; hold on; axis equal;

% Label for node
str = string(1:size(Reference,2));

% Plot initial configuration
scatter(P1_rot(1,:),P1_rot(2,:),80)
text(P1_rot(1,:)+0.15, P1_rot(2,:)+0.15, str(:,2:end));

% Plot second configuration alligned with the initial configuration
scatter(P2_rot_c_P1(1,:),P2_rot_c_P1(2,:),400,'s')
text(P2_rot_c_P1(1,:)+0.15, P2_rot_c_P1(2,:)+0.15, str(:,2:end));

% Plot trajectory of moving node
scatter(displacement(1,:), displacement(2,:),80,'red')
plot(displacement(1,:), displacement(2,:),'r--')
text(displacement(1,:)+0.15, displacement(2,:)+0.15,["P1", "P2", "P3"])

legend("$P^1_{rot}$","$P^2_{rot}$","Moving node", "Trajectory")
set(gca,'FontSize',24)

% ----------------- PLOT final -------------------------------------------
figure; hold on; axis equal;

% Label for node
str = string(1:size(Reference,2));

% Plot initial configuration
scatter(Reference(1,2:end),Reference(2,2:end),80)
text(Reference(1,2:end)+0.15, Reference(2,2:end)+0.15, str(:,2:end));

% Plot second configuration alligned with the initial configuration
scatter(map(1,:),map(2,:),400,'s')
text(map(1,:)+0.15, map(2,:)+0.15, str(:,2:end));

% Plot trajectory of moving node
scatter(displacement(1,:), displacement(2,:),80,'red')
plot(displacement(1,:), displacement(2,:),'r--')
text(displacement(1,:)+0.15, displacement(2,:)+0.15,["P0", "P1", "P2"])

legend("$Reference$","$Map$","Moving node", "Trajectory")
set(gca,'FontSize',24)

origin = [0; 0];
mdsComplete = [origin, map];
cmds = mdsComplete;

movement = shift;

end

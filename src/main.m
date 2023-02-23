
function main()
    clc;
    close all;
    clear all;
    delete(gcp('nocreate'));
    
    load exampleMaps.mat;
    N_STEP = 10;                                                             % Number of steps the the robots perform to move and capture data
    
    refMap = binaryOccupancyMap(simpleMap,1);
%     refMap = binaryOccupancyMap(complexMap);
    for i=1:8
        setOccupancy(refMap,[i 0], 1);
    end
    
%%     Creation of co2 Data concentration
    co2Map = ones(size(refMap.occupancyMatrix));
%     co2Map(10,23) = 1600;                                                 % Setting arbitrary co2 Values
%     co2Map(20,7) = 1900;
    
    co2Map(17,20) = 1600;                                                   % Setting arbitrary co2 Values
    co2Map(2,2) = 1900;

    co2Map = imgaussfilt(co2Map,10);                                        % Filter the co2 generator with a gaussian
    co2Map = co2Map/max(max(co2Map));
    co2Map = co2Map * 1900;
%     imagesc(co2Map);    
    surf(co2Map);
%%     Robots initialization

%     Initial robot locations
%     rLocations = [3 9 0 ;15 4 pi; 16 16 pi; 17 6 -pi; 22 9 pi/2; 22 22 0; 6 24 0;22 4 pi; 4 4 0];% 18 18 -pi; 16 12 0;];
    rLocations = [4 12 0; 4 16 0; 17 22 0; 22 4 pi];
    robots = {};
    for i=1:size(rLocations,1)
        id = num2str(i);
        robots{end+1} = robot(id, rLocations(i,:));
    end

    robots = updateRPositions(robots);                                      % Store the robot's positions in order to compute the distance vector
    robots = computeMds(robots, rLocations(1:end,1:2));
    robots = serialExecute(robots, refMap, co2Map, N_STEP);
    close all force;
    createGlobMap(robots, refMap.getOccupancy, co2Map);

end

%% Update the informations about the robots location in order to compute their distance
function robots = updateRPositions(robots)
    for i=1:size(robots,2)
        for j=1:size(robots,2)
            robots{i} = robots{i}.addPosition(robots{j}.id, robots{j}.absoluteLoc);
        end
    end
end

function robots = computeMds(robots, rLocations)
    robots{1} = robots{1}.computeMds(rLocations);
    for i=1:size(robots,2)
        robots{i} = robots{i}.setMds(robots{1}.getMds());
    end
end

%% Serial Robot's Execution
function robots = serialExecute(robots, refMap, co2Map, N_STEP)
    for i=1:N_STEP
        for j=1:size(robots,2)
            disp(["Robot ", robots{j}.id,": Begin STEP N.", i,"\n"]);
            robots{j} = robots{j}.scanData(refMap,co2Map);
            robots{j} = robots{j}.moveRobot();
            disp(["Robot ", robots{j}.id,": Finished STEP N.", i,"\n"]);
        end
        robots = updateRPositions(robots);
        disp(["Robot ", robots{j}.id,": Completed execution.\n"]);
        % --> Dovrebbero fare cose qui?
    end
%     robots{1}.abf.plotGraph();
end

%% Create a global map using all the robots data
function createGlobMap(robots, truthMap, truthCo2)    
    scans = [];                                                             % Concatenation of all lidar scans of all the robots
    poses = [];                                                             % Concatenation of all the poses where the scans are captured
    co2 = [];                                                               % Concatenation of all the co2 scans   
    
    a = robots{1}.rPositions;

    for i=1:size(robots,2)
        rData = robots{i}.historyData;                                      % Get the data structure that collet all the scans made from a robot
        rscan = rData.scans;                                                % Lidar scans of one robot
        rco2 = rData.co2;                                                   % Co2 Scans
        rposes = rData.poses;

        poses = cat(1,poses,rposes);
        co2 = [co2 rco2];
        scans = [scans rscan];
    end

    map = buildMap(scans,poses,3,10);                                       % Map creation
    
    mapOcc = map.getOccupancy();
    co2Map = zeros(size(mapOcc,1), size(mapOcc,2));                         % Creation an initial co2 map to update
    x = poses(:,1);
    y = poses(:,2);
    
    ij = world2grid(map,[x y]);                                             % Converting the poses of the co2 data in indices to populate the co2Matrix

%     normx = normalize(x,'range',[0 size(co2Map,1)]);
%     normy = normalize(y,'range',[0 size(co2Map,2)]);

    for i=1:size(ij,1)
        co2Map(ij(i,1),ij(i,2)) = co2(i);                                   % Populate co2 Matrix
    end

    co2Map = imgaussfilt(co2Map,10);                                        % Filter the co2 peaks with a gaussian filter
    co2Map = co2Map/max(max(co2Map));
    co2Map = co2Map * max(co2);
   
    completeMap = imfuse(mapOcc,co2Map);                                    % Fuse the area map and the co2 map in a single image
    groundTruth = imfuse(truthMap,rot90(truthCo2));                         % Complete map and co2 used for comparison 
    
    figure
    imagesc(completeMap);
    figure
    imagesc(groundTruth);
    % show(map);
end

% function createGlobMap(robots,truthMap, truthCo2)    
%     scans = [];                                                             % Concatenation of all lidar scans of all the robots
%     poses = [];                                                             % Concatenation of all the poses where the scans are captured
%     co2 = [];                                                               % Concatenation of all the co2 scans   
%     
%     a = robots{1}.rPositions;
%     gt = [];
%     for i=1:size(a,2)
%         gt = [gt a(i).loc(1,1:2)'];
%     end
% 
%     b = robots{1}.mds.values;
%     mdsNew = [];
%     for i=1:size(b,2)
%         mdsNew = [mdsNew b{i}'];
%     end
%     
%     cmds = regMds(gt,mdsNew);
% 
%     for i=1:size(robots,2)
%         rData = robots{i}.historyData;                                      % Get the data structure that collet all the scans made from a robot
%         rscan = rData.scans;                                                % Lidar scans of one robot
%         rco2 = rData.co2;                                                   % Co2 Scans
%         rposes = rData.poses;
%         rposes(:,1) = rposes(:,1) + cmds(1,i);
%         rposes(:,2) = rposes(:,2) + cmds(2,i);
% 
%         poses = cat(1,poses,rposes);
%         co2 = [co2 rco2];
%         scans = [scans rscan];
%     end
% 
%     map = buildMap(scans,poses,3,10);                                       % Map creation
%     
%     mapOcc = map.getOccupancy();
%     co2Map = zeros(size(mapOcc,1), size(mapOcc,2));                         % Creation an initial co2 map to update
%     x = poses(:,1);
%     y = poses(:,2);
%     
%     ij = world2grid(map,[x y]);                                             % Converting the poses of the co2 data in indices to populate the co2Matrix
% 
% %     normx = normalize(x,'range',[0 size(co2Map,1)]);
% %     normy = normalize(y,'range',[0 size(co2Map,2)]);
% 
%     for i=1:size(ij,1)
%         co2Map(ij(i,1),ij(i,2)) = co2(i);                                   % Populate co2 Matrix
%     end
% 
%     co2Map = imgaussfilt(co2Map,10);                                        % Filter the co2 peaks with a gaussian filter
%     co2Map = co2Map/max(max(co2Map));
%     co2Map = co2Map * max(co2);
%    
%     completeMap = imfuse(mapOcc,co2Map);                                    % Fuse the area map and the co2 map in a single image
%     groundTruth = imfuse(truthMap,rot90(truthCo2));                         % Complete map and co2 used for comparison 
%     
%     figure
%     imagesc(completeMap);
%     figure
%     imagesc(groundTruth);
%     % show(map);
% end
% 
% function P = regMds(gt, cmds)
%     gt = gt - gt(:,1);
%     cmds = cmds - cmds(:,1);
% 
%     cmdsM = cmds;
%     cmdsM(1,:) = cmdsM(1,:) * -1;
% 
%     gtTh = atan2(gt(2,1) - gt(2,2), gt(1,1) - gt(1,2));
%     cmdsTh  = atan2(cmds(2,1) - cmds(2,2), cmds(1,1) - cmds(1,2));
%     cmdsMTh  = atan2(cmdsM(2,1) - cmdsM(2,2), cmdsM(1,1) - cmdsM(1,2));
% 
%     th1 = gtTh - cmdsTh;
%     th2 = gtTh - cmdsMTh;
% 
%     R1 = [cos(th1) -sin(th1);sin(th1) cos(th1)];
%     R2 = [cos(th2) -sin(th2);sin(th2) cos(th2)];
% 
%     P1 = R1 * cmds;
%     P2 = R2 * cmdsM;
% 
%     error1 = 0;
%     error2 = 0;
%     for i = 1:size(P1,2)
%         error1 = error1 + norm(gt(:,i)- P1(:,i),2);
%         error2 = error2 + norm(gt(:,i)- P2(:,i),2);
%     end
% 
%     if error1 <= error2
%         P = P1;
%     else
%         P = P2;
%     end
% end
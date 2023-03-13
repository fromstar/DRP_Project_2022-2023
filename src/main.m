
function main()
    clc;
    close all;
    clear all;
    delete(gcp('nocreate'));
    
    load exampleMaps.mat;

    % How many scan each robot does
    N_STEP = 30; 
    
    refMap = binaryOccupancyMap(simpleMap,1);
%     refMap = binaryOccupancyMap(complexMap);

    % Closing the binary map
    for i=1:8
        setOccupancy(refMap,[i 0], 1);
    end
    
%%     Creation of co2 Data concentration
    co2Map = ones(size(refMap.occupancyMatrix));
    
    % Setting arbitrary co2 Values
    co2Map(17,20) = 1600;                                                   
    co2Map(2,2) = 1900;
    
    % Filter the co2 generator with a gaussian
    co2Map = imgaussfilt(co2Map,10);                                        
    co2Map = co2Map/max(max(co2Map));
    co2Map = co2Map * 1900;
%     imagesc(co2Map);    
%     surf(co2Map);

%%     Robots initialization

    % Initial robot locations
    rLocations = [2 8 0; 4 16 0; 17 22 0; 22 4 pi];
    robots = {};
    % Robots creation
    for i=1:size(rLocations,1)
        id = num2str(i);
        robots{end+1} = robot(id, rLocations(i,:));
    end
    
    % Send for each robot the absolute location of all the robots.
    % Just for computing the distance vector.
    robots = setRPositions(robots);

    % Compute the reference system and store it in each robot
    robots = computeMds(robots, rLocations(1:end,1:2));

    % Start the exploration
    robots = serialExecute(robots, refMap, co2Map, N_STEP);
    close all force;

    % Put together all the scans made by the robots
    createGlobMap(robots, refMap.getOccupancy, co2Map);

end

% Each robot sends its position to the other.
function robots = setRPositions(robots)
    for i=1:size(robots,2)
        for j=1:size(robots,2)
            robots{i} = robots{i}.addPosition(robots{j}.id, robots{j}.absoluteLoc);
        end
    end
end

% The first robot is used as pivot. So the first robot compute the
% MDS using itself as pivot and then send it to all the other robots
function robots = computeMds(robots, rLocations)
    robots{1} = robots{1}.computeMds(rLocations);
    for i=2:size(robots,2)
        robots{i} = robots{i}.setMds(robots{1}.getMds());
    end
end

% Update the MDS positions to all the robots
function robots = updateMDSPositions(robots)
    for i=1:size(robots,2)
        for j=1:size(robots,2)
            if i ~= j
                robots{i} = robots{i}.updateRPose(robots{j}.id, robots{j}.loc);
            end
        end
    end
end

%% Serial Robot's Execution

% Each step each robot must perform an action of scanning and an action of
% moving. When the robots finish moving, they communicate their position 
% within the updated reference system to the others.
function robots = serialExecute(robots, refMap, co2Map, N_STEP)
    for i=1:N_STEP
        for j=1:size(robots,2)
            disp(["Robot ", robots{j}.id,": Begin STEP N.", i,"\n"]);
            
            robots{j} = robots{j}.scanData(refMap,co2Map);
            robots{j} = robots{j}.moveRobot();

            disp(["Robot ", robots{j}.id,": Finished STEP N.", i,"\n"]);
        end
        robots = updateMDSPositions(robots);
        disp(["Robot ", robots{j}.id,": Completed execution.\n"]);
    end
    robots{1}.kalman.plotF();
end

%% Create a global map using all the robots data
function createGlobMap(robots, truthMap, truthCo2)    
    % Concatenation of all lidar scans of all the robots
    scans = [];
    % Concatenation of all the co2 scans   
    co2 = [];   
    % Concatenation of all the mds poses where the scans are captured
    poses = [];                                                              
    
    for i=1:size(robots,2)
        % Get the data structure that collet all the scans made from a robot
        rData = robots{i}.historyData;

        % Scans of one robot with the associated MDS position
        rscan = rData.scans;                                                
        rco2 = rData.co2;
        rposes = rData.poses;
        
        % Put together all the data of all the robots
        poses = cat(1,poses,rposes);
        co2 = [co2 rco2];
        scans = [scans rscan];
    end
    
    % Map creation
    map = buildMap(scans,poses,3,10);                                       
    
    % Creation of an initial co2 map to update
    mapOcc = map.getOccupancy();
    co2Map = zeros(size(mapOcc,1), size(mapOcc,2));                        
    x = poses(:,1);
    y = poses(:,2);
    
    % Converting the poses of the co2 data in indices to populate the co2Matrix
    ij = world2grid(map,[x y]);                                             
    
    % Populate co2 Matrix
    for i=1:size(ij,1)
        co2Map(ij(i,1),ij(i,2)) = co2(i);                                   
    end
    
    % Filter the co2 peaks with a gaussian filter
    co2Map = imgaussfilt(co2Map,10);                                        
    co2Map = co2Map/max(max(co2Map));
    co2Map = co2Map * max(co2);
    
    % Fuse the area map and the co2 map in a single image
    completeMap = imfuse(mapOcc,co2Map);

    % Complete map and co2 used for comparison 
    groundTruth = imfuse(truthMap,rot90(truthCo2));                         
    
    % Plot colored distribution of the co2
    x_grid = linspace(min(x), max(x), 100);
    y_grid = linspace(min(y), max(y), 100);
    [X,Y] = meshgrid(x_grid, y_grid);
    Z = griddata(x,y,co2,X,Y,'cubic');
    
    %% Overlay co2 data with lidar scans:

    reconMap = map.copy;

    % Get the doublexdouble matrix from the occupancy matrix
    data = reconMap.getOccupancy;

    % Offset the matrix by 1/2
    data = data - 0.5;
    
    % Round small values to zero
    [dim_data_x, dim_data_y] = size(data);
    for i = 1:dim_data_x
        for j = 1:dim_data_y
            if((data(i,j) <= 2e-4) && (data(i,j) >= -2e-4))
                data(i,j) = 0;
            end
        end
    end
    
    % Remove all empty rows and columns
    data( ~any(data,2), : ) = [];  %rows
    data( :, ~any(data,1) ) = [];  %columns
    data = data + 0.5;
    
    % Resize the explored map to match the co2 structure:
    Z_copy = Z;
    [rowsize, colsize] = size(Z_copy);
    rescaled_discoveryMap = imresize(data, [rowsize colsize]);

    % Overlay co2 and lidar scans together
    finalMap = imfuse(rescaled_discoveryMap, rot90(Z_copy));

    figure;
    contourf(X,Y,Z);
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Co2 Distribution');
    colorbar;
    
    % Plot the reconstructed map and the original map with the gas
    % distribution
    figure
    imagesc(completeMap);
    figure
    imagesc(groundTruth);
    figure
    imagesc(finalMap);
    
end

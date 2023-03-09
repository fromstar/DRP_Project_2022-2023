classdef robot
    properties
        id;                     % Robot Id
        loc;                    % Robot's mds location
        absoluteLoc;            % Robot location inside the map

        historyData;            % Collection of all data scanned by the robot
        rPositions;             % Vector of struct that contains all the known robot's positions. The info contained in the struct is the robot id and it's location
        iDistanceMatrix;        % Interdistance Matrix
        mds;                    % Multi-Dimensional Scaling
        abyf;                   % Alpha-Beta-Gamma filter
        kalman;

        diffDrive;              % Differential Drive
        controller;             % Robot Controller
        sensor;                 % Lidar Sensor
        resolution;             % Resolution of the lidar scan
    end

    methods

%% Robot constructor

        function obj = robot(idx, absLocation)
            obj.id = idx;
            obj.absoluteLoc = absLocation;                                  % Real robot's location in the map. Necessary to compute the distance from other robots
            
            obj.loc = [0 0 absLocation(3)];                                 % Starting relative location is [0,0]
            
            node.id = idx;                                                  % The initial Struct to insert in rPositions must be the robot just created.
            node.loc = absLocation;
            obj.rPositions = node;                                          % The positions of all robots are saved in a struct array.
            
            obj.mds = containers.Map;                                       % The mds is a map where the key is a robot id and the value the mds position
            obj.iDistanceMatrix = [];                                       % Interdistance Matrix
                
            obj.historyData.scans = {};                                     % All the scans captured by the robot
            obj.historyData.co2 = [];                                       % All the co2 captured by the robot
            obj.historyData.poses = [];                                     % Mds poses associated with the captured scans

            obj.abyf = ABYFilter();                                         % Alpha Beta Gamma filter for the co2 sensor
            obj.kalman = KalmanF();

            obj.diffDrive = differentialDriveKinematics("Trackwidth", ...   % Differential-drive vehicle model to simulate simplified vehicle dynamics
                1,"VehicleInputs","VehicleSpeedHeadingRate");

            obj.controller = controllerPurePursuit;                         % Controller object used to make a differential-drive vehicle follow a set of waypoints.
            obj.controller.DesiredLinearVelocity = 1;
            obj.controller.MaxAngularVelocity = 2;
            obj.controller.LookaheadDistance = 0.6;

            obj.sensor = rangeSensor;                                       % Sensor used for capturing lidar scan
            obj.sensor.Range = [0 20];
            obj.resolution = 3;
%             obj.reg = regularizator_mds;
        end

%% Update other robot's position

        function obj = addPosition(obj, idx, location)
            i = 1;
            inserted = false;
            while i <= size(obj.rPositions, 2) && inserted == false           % Look if the robot to insert already exist. If yes update update it's position.
                if obj.rPositions(i).id == idx
                    obj.rPositions(i).loc = location;
                    inserted = true;
                end
                i = i+1;
            end

            if inserted == false                                            % If the robot is not there then it must be inserted
                node.id = idx;
                node.loc = location;
                obj.rPositions = [obj.rPositions node];
            end
        end

%% Compute interdistance vector from the received robot to the others using the known positions.
        function iDistanceVector = computeIDistanceVector(obj, aloc)
            iDistanceVector = zeros(size(obj.rPositions));
            for i=1:size(obj.rPositions,2)                                  
                rloc = obj.rPositions(i);
%                 iDistanceVector(i) = (norm(aloc.loc(1:2) - ...
%                                             rloc.loc(1:2), 2))^2;
                iDistanceVector(i) = (norm(aloc.loc(1:2) - rloc.loc(1:2), 2));
            end
        end

%% Store others interdistance vector of other robots
        function [obj,iDistMat] = computeIDistanceMatrix(obj)
            iDistMat = zeros([size(obj.rPositions,2)]);
            
            for i=1:size(obj.rPositions,2)
                distVect = obj.computeIDistanceVector(obj.rPositions(i));
                iDistMat(i,:) = distVect;
            end
            obj.iDistanceMatrix = iDistMat;
        end

%% Clear interdistance matrix
        function obj = resetIDistanceMatrix(obj)
            obj.iDistanceMatrix = [];
        end

%% Get the mds coordinates
function [obj,rmds] = computeMds(obj, gt)
            
            mds = containers.Map;
            [obj,distanceMatrix] = obj.computeIDistanceMatrix();
%             symMds = maccgen(distanceMatrix)'; 
%             rmds = zeros(size(symMds));
%             for i=1:size(rmds,1)                                            % The mds returned is symbolic so it must be converted.
%                 for j=1:size(rmds,2)
%                     val = sym2cell(symMds(i,j));
%                     rmds(i,j) = val{1};
%                 end
%             end

            [rmds, eigen] = cmdscale(obj.iDistanceMatrix);            
            
            reference = [];
            theta = [];
            for i=1:size(obj.rPositions,2)
                reference = [reference; obj.rPositions(i).loc(1:2)]
                theta = [theta; obj.rPositions(i).loc(3)]
            end

            [rmds, shift] = main_2(reference', theta', size(obj.rPositions,2));
            
            rmds = rmds';
%             obj.absoluteLoc(1:2) = obj.absoluteLoc(1:2) + shift';
            obj.absoluteLoc(1:2) = obj.absoluteLoc(1:2);

            for i=1:size(obj.rPositions,2)
                if obj.rPositions(i).id == obj.id
                    break;
                end
            end

            rmds = obj.regMds(rmds',gt',i)';

            for i=1:size(rmds,1)                                            % The mds returned is symbolic so it must be converted.
                mds(obj.rPositions(i).id) = rmds(i,:);
            end
            obj.mds = mds; 
        end

        function obj = setMds(obj,mds)
            obj.mds = mds;
            loc = mds(obj.id);
            obj.loc(1) = loc(1);
            obj.loc(2) = loc(2);
        end

        function mds = getMds(obj)
            mds = obj.mds;
        end

        function P = regMds(obj, cmds, gt, i)
            gt = gt - gt(:,i);
            cmds = cmds - cmds(:,i);
        
            cmdsM = cmds;
            cmdsM(1,:) = cmdsM(1,:) * -1;
        
            gtTh = atan2(gt(2,1) - gt(2,2), gt(1,1) - gt(1,2));
            cmdsTh  = atan2(cmds(2,1) - cmds(2,2), cmds(1,1) - cmds(1,2));
            cmdsMTh  = atan2(cmdsM(2,1) - cmdsM(2,2), cmdsM(1,1) - cmdsM(1,2));
        
            th1 = gtTh - cmdsTh;
            th2 = gtTh - cmdsMTh;
        
            R1 = [cos(th1) -sin(th1);sin(th1) cos(th1)];
            R2 = [cos(th2) -sin(th2);sin(th2) cos(th2)];
        
            P1 = R1 * cmds;
            P2 = R2 * cmdsM;
        
            error1 = 0;
            error2 = 0;
            for i = 1:size(P1,2)
                error1 = error1 + norm(gt(:,i)- P1(:,i),2);
                error2 = error2 + norm(gt(:,i)- P2(:,i),2);
            end
        
            if error1 <= error2
                P = P1;
            else
                P = P2;
            end
        end

%% Scan the information of the environment

        function [obj,scan] = scanArea(obj, refMap)
            while obj.absoluteLoc(3) > 2*pi                                 % Set the robot orientation in range [0,2pi]
                obj.absoluteLoc(3) = obj.absoluteLoc(3) - 2*pi;
            end
            while obj.absoluteLoc(3) < 0
                obj.absoluteLoc(3) = obj.absoluteLoc(3) + 2*pi;
            end

            [ranges, angles] = obj.sensor(obj.absoluteLoc, refMap);         % Read data from lidar sensor
            scan = lidarScan(ranges, angles);
        end

%% Read co2 data concentration
        
        function [obj, meas] = scanCo2(obj, co2Map)
            i = round(obj.absoluteLoc(1));                                  % Approximate robot's xy coordinates to the nearest integer so as to have
            j = round(obj.absoluteLoc(2));                                  % the indices of the position to be used in the co2 matrix
            lowValue =  co2Map(i,j) - 10;                                   % Get a random value around the true ppm co2
            highValue = co2Map(i,j) + 10;                                   % This is done in order to simulate the measure error of a sensor

            for k=1:100                                                      % Read 40 data to regulate the Alha-Beta Filter;           
                nmeas = unifrnd(lowValue,highValue);                        % Read a noisy measurement
                [obj.abyf,meas] = obj.abyf.updateF(nmeas,co2Map(i,j));    % Filter the co2 data using an Alpha Beta Gamma Filter
                obj.kalman = obj.kalman.updateF(nmeas,co2Map(i,j));
            end
            obj.abyf.plotF();
            obj.kalman.plotF();
            obj.kalman = obj.kalman.resetF();
            obj.abyf = obj.abyf.reset();                                    % After the robot moved the co2 data might be very different. 
        end                                                                 % Due to this it's necessary to reset the sensor in order to avoid that the filter need too much data to regulate itself

%% General scan function that stores the data in a proper struct

        function [obj] = scanData(obj,refMap,co2Map)
            [obj,scan] = obj.scanArea(refMap);                              % Get the lidar scan
            [obj,co2] = obj.scanCo2(co2Map);                                % Get the co2 concentration from sensor

            scans = obj.historyData.scans;                                  % Get all the previous lidar scans captured by the robot
            co2s =  obj.historyData.co2;                                    % Get all the previous co2 scans captured by the robot
            poses = obj.historyData.poses;                                  % Get all the poses associated with the captured scans

            scans{end+1} = scan;                                            % Concatenate the last lidar scan captured
            co2s(end+1) = co2;                                              % Concatenate the last co2 scan captured
            poses(end+1,:) = obj.loc;                                       % Concatenate the pose associated with the last scan captured

            obj.historyData.scans = scans;                                  % Updated the robot's struct
            obj.historyData.co2 = co2s;
            obj.historyData.poses = poses;
        end

%% Return a struct that contains the information of the last scan
        function lastScan = getLastScan(obj)
            lastScan.id = obj.id;
            lastScan.pose = obj.historyData.poses(end);
            lastScan.scan = obj.historyData.scans{end};
            lastScan.co2 = obj.historyData.co2(end);
        end

        function obj = clearData(obj)
            obj.historyData.poses = [];
            obj.historyData.scans = {};
            obj.historyData.co2 = [];
        end

%% Plot co2 measurements
        function plotFilterGraph(obj)
            obj.abyf.plotGraph();
        end

%% Chose a goal position where to move and compute the path

        function [obj,path] = pickPath(obj)
            
            disp(["Robot " obj.id ": Choosing path"]);
            scans = obj.historyData.scans;
            poses = obj.historyData.poses;
            res = obj.resolution;
            range = obj.sensor.Range(2);
            map = buildMap(scans, poses,res,range);                         % Build a local map using the data captured by the single robot
            inflateMap = map;
            inflate(inflateMap,obj.diffDrive.TrackWidth/20);
            if(~checkOccupancy(inflateMap,obj.loc(1:2)))
                map = inflateMap;
            end
            map.OccupiedThreshold = 0.7;            
            map.FreeThreshold = 0.5;                                       % 0.5 is the value used for the unseen area. So it's probably that that space is free since the scan might be scanned but the wall is very distant
            
            figure
            hold on
            show(map);
            set(gca, "XLim", [-50 50]);
            hold all

            xLocalLimits = [-map.XLocalLimits(2) ...
                map.XLocalLimits(2)];
            yLocalLimits = [-map.YLocalLimits(2) ...
                map.YLocalLimits(2)];
            theta = [obj.loc(3)-pi obj.loc(3)+pi];

            space = stateSpaceDubins([xLocalLimits; ...                     % Create a space where select a goal point
                yLocalLimits; theta]);
            validator = validatorOccupancyMap(space);                       % Validate the space
            validator.Map = map();                                          % Set the map for the validator
            validator.ValidationDistance = 0.2;
            
            planner = plannerAStarGrid(map);                      
%             planner = plannerRRTStar(space,validator);                    % Using RRTStar for the planner
%             planner.MaxNumTreeNodes = 1000;   
%             planner.MaxConnectionDistance = 0.7;
%             planner.ContinueAfterGoalReached = true;
%             planner.MaxIterations = 10000;
            
            startLocation = obj.loc;                                        % Set starting position of the robots            
            cnt = 0;
            costTs = 4;
            while(true)        
               while(true)                                                  % Continue to pick up a goal position until this is in a free space
                    Radius = 2;                                             % Select a random goal around the robot    
                    r = Radius * (sqrt(rand)+0.5);
                    if(cnt < 500)
                        minTh = obj.loc(3) - pi/2;
                        maxTh = obj.loc(3) + pi/2;
                        theta = minTh + (maxTh-minTh).*rand(1,1);
                    else
                        Radius = Radius + 1;                                             % Select a random goal around the robot    
                        r = Radius * (sqrt(rand)+0.5);
                        theta = rand * 2 * pi;
                        costTs = costTs + 1;
                    end
                    
                    x = obj.loc(1) + r * cos(theta);
                    y = obj.loc(2) + r * sin(theta);

                    endLocation = [x y theta];                              % Check if the random goal is in a valid state not occupied
                    occMatrix = checkOccupancy(map,endLocation(1:2)-1 ,[1 1]);
                    if(validator.isStateValid(endLocation) && ~ismember(1,occMatrix))
                        break;
                    end
               end

               start = world2grid(map,startLocation(1:2));
               goal = world2grid(map,endLocation(1:2));
               [path,info] = plan(planner,start,goal);
               info
               for i=1:size(path,1)
                   path(i,:) = grid2world(map,path(i,:));
               end
               path = [path zeros(size(path,1),1)];
               path(1,3) = startLocation(3);
               for i=2:size(path,1)
                   path(i,3) = atan2(path(i,2)-path(i-1,2),path(i,1)-path(i-1,1));
               end
               if (~isempty(path) && info.PathCost < costTs)
                  break;
               end
               cnt = cnt+1;
            end
%                     disp("End Location is distant from wall\n");
%                     disp(["robot pose: " obj.loc " goal: " endLocation]);
%                     [pthObj,solnInfo] = plan(planner,startLocation, ...     % Get the plan to the random goal
%                         endLocation);
%                     pathMetricsObj = pathmetrics(pthObj,validator);
%                     
%                     if size(pathMetricsObj.Path.States,1) >= 2              % Check if the path has 2 or more points. If not the robot doesn't move!
%                         isValid = true;
%                         i = 1;
%                         while i<size(pathMetricsObj.Path.States,1) && isValid % Check for each pair of path's points if the path is motion valid
%                             state1 = pathMetricsObj.Path.States(i,:);
%                             state2 = pathMetricsObj.Path.States(i+1,:);
%                             
%                             th = state2(3);
%                             while ~isMotionValid(validator,state1,state2)... % Maybe for some angles the path is not valid but for other it is
%                                 && state2(3) < th+(2*pi)
%                                 state2(3) = state2(3) + 0.1;
%                             end
%                             if ~isMotionValid(validator,state1,state2)
%                                 isValid = false;
%                             end
%                             i=i+1;
%                         end
%                         if isValid
%                             path = pathMetricsObj.Path.States;              % If a valid path is found exit the loop.
%                             break;
%                         end
%                     end
%                 end

            disp(["Robot ", obj.id,": Path to follow chosen.\nStarting to move...\n"]);
            map.show
            hold on
%             plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
            plot(path(:,1),path(:,2),'r-','LineWidth',2)
        end

%% Move the robot from a start position to a goal position

        function obj = moveRobot(obj)
            [obj,path] = obj.pickPath();                         % Get a random goal position and the path to reach it
            scans = obj.historyData.scans;
            poses = obj.historyData.poses;
            res = obj.resolution;
            range = obj.sensor.Range(2);
            map = buildMap(scans, poses,res,range);

            initialLocation = path(1,:);
            robotGoal = path(end,:);
            path = path(:,1:end-1);
            
            sampleTime = 0.1;
            vizRate = rateControl(1/sampleTime);

            release(obj.controller);
            obj.controller.Waypoints = path;

            robotCurrentPose = initialLocation';
            distanceToGoal = norm(initialLocation - robotGoal);
            goalRadius = 0.1;                                               % Distance tolerance radius to consider that the
                                                                            % robot has reached the goal position
            reset(vizRate);

            % Initialize the figure
            figure
            frameSize = obj.diffDrive.TrackWidth/0.8;

            while( distanceToGoal > goalRadius )

                % Compute the controller outputs, i.e., the inputs to the robot
                [v, omega] = obj.controller(robotCurrentPose);

                % Get the robot's velocity using controller inputs
                vel = derivative(obj.diffDrive, robotCurrentPose, [v omega]);

                % Update the current pose
                robotCurrentPose = robotCurrentPose + vel*sampleTime;

                % Re-compute the distance to the goal
                distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(1:2)');

                % Update the plot
                hold off
                show(map);
                set(gca, "XLim", [-50 50]);
                hold all

                % Plot path each instance so that it stays persistent while robot mesh
                % moves
                plot(path(:,1), path(:,2),"k--d")

                % Plot the path of the robot as a set of transforms
                plotTrVec = [robotCurrentPose(1:2); 0];
                plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
                plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
                light;
                xlim([-20 20])
                ylim([-20 20])

                waitfor(vizRate);
            end
            robotCurrentPose = robotCurrentPose';
            x = robotCurrentPose(1);
            y = robotCurrentPose(2);
            th = robotCurrentPose(3);
            
            rMovement = [x - obj.loc(1), y - obj.loc(2), 0];
            
            obj.absoluteLoc = obj.absoluteLoc + rMovement;
            obj.absoluteLoc(3) = th;

            obj.addPosition(obj.id, obj.absoluteLoc);

            obj.loc = [x,y,th];
            close all force;
            disp(["Robot ", obj.id,": End position reached, waiting for new command.\n"]);
        end
    end
end
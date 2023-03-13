classdef robot
    properties
        id;                     % Robot Id
        mdsLoc;                    % Robot's mds location
        absoluteLoc;            % Robot location inside the map

        historyData;            % Collection of all data scanned by the robot
        rPositions;             % Vector of struct that contains all the known robot's positions. The info contained in the struct is the robot id and it's location
        iDistanceMatrix;        % Interdistance Matrix
        mds;                    % Multi-Dimensional Scaling
        kalman;                 % Kalman Filter

        diffDrive;              % Differential Drive
        controller;             % Robot Controller
        sensor;                 % Lidar Sensor
        resolution;             % Resolution of the lidar scan
    end

    methods

%% Robot constructor

        function obj = robot(idx, absLocation)
            
            % Robot id
            obj.id = idx;

            % Real robot's location in the map. 
            % Necessary to compute the distance from other robots
            obj.absoluteLoc = absLocation;                                  
            
            % Robot position in the mds reference system. 
            % Just initialized to [0,0]
            obj.mdsLoc = [0 0 absLocation(3)];                                 
            
            % Each robot keeps track of all the global positions using a
            % struct.
            node.id = idx;                                                  
            node.loc = absLocation;
            % The positions of all robots are saved in a struct array.
            obj.rPositions = node;                                          
            
            % The mds is a map structure where the key is a robot id 
            % and the value the mds position
            obj.mds = containers.Map;                                       
                
            % All the scans captured by the robot
            obj.historyData.scans = {};  
            % All the co2 captured by the robot
            obj.historyData.co2 = [];   
            % Mds poses associated with the captured scans
            obj.historyData.poses = [];                                     
            
            % Kalman Filter
            obj.kalman = KalmanF();
            
            % Differential-drive vehicle model to 
            % simulate simplified vehicle dynamics
            obj.diffDrive = differentialDriveKinematics("Trackwidth", ...   
                1,"VehicleInputs","VehicleSpeedHeadingRate");
            
            % Controller object used to make a differential-drive 
            % vehicle follow a set of waypoints.
            obj.controller = controllerPurePursuit;                         
            obj.controller.DesiredLinearVelocity = 1;
            obj.controller.MaxAngularVelocity = 2;
            obj.controller.LookaheadDistance = 0.6;
            
            % Sensor used for capturing lidar scan
            obj.sensor = rangeSensor;                                       
            obj.sensor.Range = [0 20];
            obj.resolution = 3;
        end

        % Insert other robot's global position
        function obj = addPosition(obj, idx, location)
            i = 1;
            inserted = false;
            % Look if the robot to insert already exists. 
            % If yes update update its position.
            while i <= size(obj.rPositions, 2) && inserted == false           
                if obj.rPositions(i).id == idx
                    obj.rPositions(i).loc = location;
                    inserted = true;
                end
                i = i+1;
            end
           
            %  If the robot is not there then it must be inserted
            if inserted == false                                            
                node.id = idx;
                node.loc = location;
                obj.rPositions = [obj.rPositions node];
            end
        end

%% MDS FUNCTIONS

        % Get the mds coordinates
        function [obj,rmds] = computeMds(obj, gt)
            
            mds = containers.Map;
            
            % Build a vector with all the robots xy positions
            reference = [];
            for i=1:size(obj.rPositions,2)
                reference = [reference; obj.rPositions(i).loc(1:2)]
            end

            [rmds, shift] = main_2(reference', obj.absoluteLoc(3), size(obj.rPositions,2));           
            close all;
            obj.absoluteLoc(1) = obj.absoluteLoc(1) + shift(1);
            obj.absoluteLoc(2) = obj.absoluteLoc(2) + shift(2);
            % To disambiguate the mds the robot used as pivot move along 2
            % axes. So the it would be not anymore at 0,0 but it's new mds
            % position would coincide with the translation along xy.
            obj.mdsLoc(1) = shift(1);
            obj.mdsLoc(2) = shift(2);
            rmds = rmds';

            % Take the index of the robot that compute the MDS in its
            % robots list
            for i=1:size(obj.rPositions,2)
                if obj.rPositions(i).id == obj.id
                    break;
                end
            end
            
            % Rototraslate the mds in order to get positions aligned with
            % the matlab simulation
            rmds = obj.regMds(rmds',gt',i)';
            
            % Build a map where a robot's id is the key and the position
            % the value
            for i=1:size(rmds,1)                                            
                mds(obj.rPositions(i).id) = rmds(i,:);
            end
            obj.mds = mds; 
        end
        
        % Save a received MDS and the one's own position
        function obj = setMds(obj,mds)
            obj.mds = mds;
            loc = mds(obj.id);
            obj.mdsLoc(1) = loc(1);
            obj.mdsLoc(2) = loc(2);
        end
        
        % Return the mds
        function mds = getMds(obj)
            mds = obj.mds;
        end

        function P = regMds(obj, cmds, gt, i)
            % Use just the xy positions without theta
            gt = gt - gt(:,i);
            cmds = cmds - cmds(:,i);
            
            % Mirror the mds
            cmdsM = cmds;
            cmdsM(1,:) = cmdsM(1,:) * -1;
            
            % Angle formed by the first point with the second for the
            % ground truth, the MDS and the Mirrored MDS
            gtTh = atan2(gt(2,1) - gt(2,2), gt(1,1) - gt(1,2));
            cmdsTh  = atan2(cmds(2,1) - cmds(2,2), cmds(1,1) - cmds(1,2));
            cmdsMTh  = atan2(cmdsM(2,1) - cmdsM(2,2), cmdsM(1,1) - cmdsM(1,2));
            
            % How much rotate the MDS positions
            th1 = gtTh - cmdsTh;
            th2 = gtTh - cmdsMTh;
            
            % Rotation Matrix
            R1 = [cos(th1) -sin(th1);sin(th1) cos(th1)];
            R2 = [cos(th2) -sin(th2);sin(th2) cos(th2)];
            
            % Positions rotation
            P1 = R1 * cmds;
            P2 = R2 * cmdsM;
            
            % Compute how much the positions are distant from the ground
            % truth.
            error1 = 0;
            error2 = 0;
            for i = 1:size(P1,2)
                error1 = error1 + norm(gt(:,i)- P1(:,i),2);
                error2 = error2 + norm(gt(:,i)- P2(:,i),2);
            end
            
            % Store the rotated system with the min error
            if error1 <= error2
                P = P1;
            else
                P = P2;
            end
        end
        
        % Update the mds position of a certain robot
        function obj = updateRPose(obj,idx,pose)
            obj.mds(idx) = pose;
        end

%% SCAN FUNCTIONS

        % Scan the information of the environment
        function [obj,scan] = scanArea(obj, refMap)
            % Read data from lidar sensor
            [ranges, angles] = obj.sensor(obj.absoluteLoc, refMap);         
            scan = lidarScan(ranges, angles);
        end
        
        % Read co2 data concentration
        function [obj, meas] = scanCo2(obj, co2Map)
            % Approximate robot's xy coordinates to the nearest integer so as to have
            % the indices of the position to be used in the co2 matrix
            i = round(obj.absoluteLoc(1));                              
            j = round(obj.absoluteLoc(2));   

            % Get a random value around the true ppm co2
            % This is done in order to simulate the measure error of a sensor
            lowValue =  co2Map(i,j) - 10;                                   
            highValue = co2Map(i,j) + 10;                                   
            
            % Read 30 data to regulate the Kalman; 
            for k=1:100            
                % Read a noisy measurement
                nmeas = unifrnd(lowValue,highValue);     
                % Filter the read measure
                [obj.kalman,meas] = obj.kalman.updateF(nmeas,co2Map(i,j));
            end
            obj.kalman.plotG();
            obj.kalman.plotF();
             % After the movement the co2 value should be different.
             % Resetting the filter accelerate the output of a good value
            obj.kalman = obj.kalman.resetF();
        end                                                                

        % General scan function that stores the data in a proper struct
        function [obj] = scanData(obj,refMap,co2Map)
            % Get the lidar scan
            [obj,scan] = obj.scanArea(refMap);
            % Get the co2 concentration from sensor
            [obj,co2] = obj.scanCo2(co2Map);                                
            
            % Get all the previous lidar scans captured by the robot
            scans = obj.historyData.scans; 
            % Get all the previous co2 scans captured by the robot
            co2s =  obj.historyData.co2; 
            % Get all the poses associated with the captured scans
            poses = obj.historyData.poses;                                  
            
            % Concatenate the last lidar scan captured
            scans{end+1} = scan;  
            % Concatenate the last co2 scan captured
            co2s(end+1) = co2;       
            % Concatenate the pose associated with the last scan captured
            poses(end+1,:) = obj.mdsLoc;                                       
            
            % Updated the robot's struct
            obj.historyData.scans = scans;                                  
            obj.historyData.co2 = co2s;
            obj.historyData.poses = poses;
        end

        % Return a struct that contains the information of the last scan
        function lastScan = getLastScan(obj)
            lastScan.id = obj.id;
            lastScan.pose = obj.historyData.poses(end);
            lastScan.scan = obj.historyData.scans{end};
            lastScan.co2 = obj.historyData.co2(end);
        end
        
        % Delete all the scans made
        function obj = clearData(obj)
            obj.historyData.poses = [];
            obj.historyData.scans = {};
            obj.historyData.co2 = [];
        end

        % Plot co2 measurements
        function plotFilterGraph(obj)
            obj.kalman.plotF();
        end

%% Movement Functions
        
        % Chose a goal position where to move and compute the path
        function [obj,path] = pickPath(obj)
            
            disp(["Robot " obj.id ": Choosing path"]);
            % Build a local map with the information of just one robot
            scans = obj.historyData.scans;
            poses = obj.historyData.poses;
            res = obj.resolution;
            range = obj.sensor.Range(2);
            map = buildMap(scans, poses,res,range);              
            inflateMap = map;
            inflate(inflateMap,obj.diffDrive.TrackWidth/20);
            if(~checkOccupancy(inflateMap,obj.mdsLoc(1:2)))
                map = inflateMap;
            end
            map.OccupiedThreshold = 0.7;
            % 0.5 is the value used for the unseen area. So it's probably 
            % that that space is free since the scan might be scanned but 
            % the wall is very distant
            map.FreeThreshold = 0.5;                                       
            
            figure
            hold on
            show(map);
            set(gca, "XLim", [-50 50]);
            hold all

            xLocalLimits = [-map.XLocalLimits(2) ...
                map.XLocalLimits(2)];
            yLocalLimits = [-map.YLocalLimits(2) ...
                map.YLocalLimits(2)];
            theta = [obj.mdsLoc(3)-pi obj.mdsLoc(3)+pi];
            
            % Create a space where select a goal point
            space = stateSpaceDubins([xLocalLimits; ...                     
                yLocalLimits; theta]);
            % Validate the space
            validator = validatorOccupancyMap(space);
            % Set the map for the validator
            validator.Map = map();                                          
            validator.ValidationDistance = 0.2;
            
            % Set planner
            planner = plannerAStarGrid(map);                      
            
            % Set starting position of the robots
            startLocation = obj.mdsLoc;                                        
            cnt = 0;
            
            % Continue to pick up a goal position until the path is
            % feasible
            while(true)   
               % Continue to pick a random goal around the robot until this
               % is in free space
               while(true)
                    % Picking a random distance goal
                    Radius = 2;                                             
                    r = Radius * (sqrt(rand)+0.5);
                    % Picking a random theta for the goal position
                    % Trying to pick an angle in front of the robot, if not
                    % possible expands the search area to 2Pi
                    if(cnt < 500)
                        minTh = obj.mdsLoc(3) - pi/2;
                        maxTh = obj.mdsLoc(3) + pi/2;
                        theta = minTh + (maxTh-minTh).*rand(1,1);
                    else
                        Radius = Radius + 1;                                             % Select a random goal around the robot    
                        r = Radius * (sqrt(rand)+0.5);
                        theta = rand * 2 * pi;
                    end
                    
                    % Conversion the goal position in xt coordinates
                    x = obj.mdsLoc(1) + r * cos(theta);
                    y = obj.mdsLoc(2) + r * sin(theta);
                    
                    % Check if the random goal is in a valid state not
                    % occupied and distant from walls
                    endLocation = [x y theta];                              
                    occMatrix = checkOccupancy(map,endLocation(1:2)-1 ,[1 1]);
                    if(validator.isStateValid(endLocation) && ~ismember(1,occMatrix))
                        break;
                    end
               end
               
               % Conversion of start and goal positions in grid indices
               start = world2grid(map,startLocation(1:2));
               goal = world2grid(map,endLocation(1:2));
               % Get the path from the planner
               [path,info] = plan(planner,start,goal);
               for i=1:size(path,1)
                   path(i,:) = grid2world(map,path(i,:));
               end
               % Add the arriving theta for each point in the path
               path = [path zeros(size(path,1),1)];
               % The starting theta should be the robot orientation
               path(1,3) = startLocation(3);
               % The arriving theta in the other points is choosen as the
               % angle formed between 2 sequential points.
               for i=2:size(path,1)
                   path(i,3) = atan2(path(i,2)-path(i-1,2),path(i,1)-path(i-1,1));
               end
               if (~isempty(path))
                  break;
               end
               cnt = cnt+1;
            end

            disp(["Robot ", obj.id,": Path to follow chosen.\nStarting to move...\n"]);
            map.show
            hold on
            plot(path(:,1),path(:,2),'r-','LineWidth',2)
        end

%% Move the robot from a start position to a goal position

        function obj = moveRobot(obj)
            % Get a random goal position and the path to reach it
            [obj,path] = obj.pickPath();        
            % Build a local map with a robot information to visualize the
            % robot movement
            scans = obj.historyData.scans;
            poses = obj.historyData.poses;
            res = obj.resolution;
            range = obj.sensor.Range(2);
            map = buildMap(scans, poses,res,range);
            
            % Initialize the starting position
            initialLocation = path(1,:);
            robotGoal = path(end,:);
            path = path(:,1:end-1);
            
            sampleTime = 0.1;
            vizRate = rateControl(1/sampleTime);

            release(obj.controller);
            obj.controller.Waypoints = path;

            robotCurrentPose = initialLocation';
            distanceToGoal = norm(initialLocation - robotGoal);

            % Distance tolerance radius to consider that the robot has 
            % reached the goal position
            goalRadius = 0.15;  
                                                                                
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
                set(gca, "XLim", [-70 70]);
                hold all

                % Plot path each instance so that it stays persistent while robot mesh
                % moves
                plot(path(:,1), path(:,2),"k--d")

                % Plot the path of the robot as a set of transforms
                plotTrVec = [robotCurrentPose(1:2); 0];
                plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
                plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
                light;
                xlim([-30 30])
                ylim([-30 30])

                waitfor(vizRate);
            end

            robotCurrentPose = robotCurrentPose';
            x = robotCurrentPose(1);
            y = robotCurrentPose(2);
            th = robotCurrentPose(3);
            
            % How much a robot moved along xy axes
            rMovement = [x - obj.mdsLoc(1), y - obj.mdsLoc(2), 0];
            
            % Update the robot's absolute location
            obj.absoluteLoc = obj.absoluteLoc + rMovement;
            obj.absoluteLoc(3) = th;

            obj.addPosition(obj.id, obj.absoluteLoc);
            
            % Update the robot's mds position after the movement
            obj.mdsLoc = [x,y,th];
            close all force;
            disp(["Robot ", obj.id,": End position reached, waiting for new command.\n"]);
        end
    end
end
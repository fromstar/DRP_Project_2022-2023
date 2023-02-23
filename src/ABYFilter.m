classdef ABYFilter
    properties
        alpha;
        beta;
        gamma;
        accelerations
        velocities;
        trueValues;
        measurements;
        noisyMeasurements;
        errors;
        predictions;
        history;
    end

    methods
        function obj = ABYFilter()
            % NOTE: The alpha value is set very low because the co2 is read in
            % a randomic way. In a realistic it must be set with an higher
            % value since a very precise sensor would be used.
            % Low alpha -> The prediction is more important than the
            %              readings.
            % High alpha-> The sensor reading is more important than the
            %              prediction
            % Same thing for Beta and the velocity of the system.

            obj.alpha = 0.1;                     % Factor that weighs the error of the value prediction when we go to calculate the true value
            obj.beta = 0.02;                     % Factor that weighs the error of the velocity prediction when we go to calculate the true value
            obj.gamma = 0.01;
            obj.accelerations = [];              % Acceleration of the system
            obj.velocities = [];                 % Velocities of the system
            obj.trueValues = [];                 % True values of the measurements. Necessary only for plots.
            obj.measurements = [];               % Filtered values
            obj.noisyMeasurements = [];          % Noisy measurements
            obj.errors = [];                     % Error between the expected measurements and the noisy measurements
            obj.predictions = [];                % Predicted measurements
            
            obj.history.velocities=[];
            obj.history.accelerations=[];
            obj.history.trueValues = [];                 % True values of the measurements. Necessary only for plots.
            obj.history.measurements = [];               % Filtered values
            obj.history.noisyMeasurements = [];          % Noisy measurements
            obj.history.errors = [];                     % Error between the expected measurements and the noisy measurements
            obj.history.predictions = [];            
        end

%% Filter the noisy measurement
        function [obj,meas] = updateF(obj, noisyMeas, trueValue, dt)

            if isempty(obj.velocities)                                      % The first execution initialize the values necessary for the prediction
                v = 0;                                                      % The first iteration is not possible say if the noisyMeasure is correct or not. So assume it correct only for this time
                a = 0;
                meas = noisyMeas;                                           
            else
                expectedMeas = obj.predict(dt);                       % Get the prediction of the actual expected measure
                error = noisyMeas - expectedMeas;                           % Expected error
                
                lastV = obj.velocities(end);
                lastA = obj.accelerations(end);
                
                v = lastV + (obj.beta * (error / dt));                      % The system velocity is given by the last velocity seen plus the weighted error for a beta scalar over dt
                a = lastA + (obj.gamma * (error/(0.5*(dt^2))));
                meas = expectedMeas + obj.alpha * error;                    % The measure is given by the expected measure plus the weighted error for an alpha scalar


                obj.errors(end+1) = error;                                  % Store data
                obj.predictions(end+1) = expectedMeas;
                obj.trueValues(end+1) = trueValue;
                obj.noisyMeasurements(end+1) = noisyMeas;
            end
            obj.accelerations(end+1) = a;
            obj.velocities(end+1) = v;
            obj.measurements(end+1) = meas;
        end

%% Guess of the next value  
        function prediction =  predict(obj, dt)           
            lastV = obj.velocities(end);
            lastA = obj.accelerations(end);
            prediction = obj.measurements(end) + (dt * lastV) + ...
                lastA*((dt^2)/2);
        end

        function obj = reset(obj)

            obj.history.velocities=[obj.history.velocities obj.velocities];
            obj.history.accelerations=[obj.history.accelerations obj.accelerations];
            obj.history.trueValues = [obj.history.trueValues obj.trueValues];               
            obj.history.measurements = [obj.history.measurements obj.measurements(2:end)];               
            obj.history.noisyMeasurements = [obj.history.noisyMeasurements obj.noisyMeasurements];       
            obj.history.errors = [obj.history.errors obj.errors];                     
            obj.history.predictions = [obj.history.predictions obj.predictions];  

            obj.accelerations = [];              % Acceleration of the system
            obj.velocities = [];                 % Velocities of the system
            obj.trueValues = [];                 % True values of the measurements. Necessary only for plots.
            obj.measurements = [];               % Filtered values
            obj.noisyMeasurements = [];          % Noisy measurements
            obj.errors = [];                     % Error between the expected measurements and the noisy measurements
            obj.predictions = [];
        end

%% Plot the measurements graph
        function plotGraph(obj)
            figure
            x = uint32(1):uint32(size(obj.trueValues,2) + size(obj.history.trueValues,2));
            y = [obj.history.trueValues obj.trueValues];
            plot(x,y,'-g')
            
            hold on 
            y2 = [obj.history.noisyMeasurements obj.noisyMeasurements];
            plot(x,y2,'-r')
            
            y3 = [obj.history.predictions obj.predictions];
            plot(x,y3,'-k')

            y4 = [obj.history.measurements obj.measurements(2:end)];
            plot(x,y4,'-b')

            legend('True Values','Noisy Measurements', 'Predicted Values' ,'Filtered Values')
            hold off
        end
    end
end
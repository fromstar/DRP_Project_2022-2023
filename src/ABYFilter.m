classdef ABYFilter
    properties
        alpha;
        beta;
        gamma;
        
        x;
        dx;
        ddx;
        dt;
        
        noisy_ppm
        filtered_ppm
        true_ppm
        predicted

        history
    end

    methods
        function obj = ABYFilter()


            obj.alpha = 0.3;                  
            obj.beta = 0.1;
            obj.gamma = 0.1;

            obj.x = 0; % system estimation
            obj.dx = 0;
            obj.ddx = 0;
            obj.dt = 1;

            obj.filtered_ppm = [];
            obj.true_ppm = [];
            obj.noisy_ppm = [];
            obj.predicted = [];

            obj.history.noisy_ppm = [];
            obj.history.filtered_ppm = [];
            obj.history.true_ppm = [];
            obj.history.predicted = [];
        end

%% Filter the noisy measurement
        function [obj,meas] = updateF(obj, noisyMeas, trueValue)
            close all;
            if(obj.x == 0)
                obj.x = noisyMeas;
                x_est = noisyMeas;
            else
                x_est = obj.x + (obj.dt*obj.dx) + (obj.ddx*((obj.dt^2)/2));

                err = noisyMeas - x_est;

                obj.x = obj.x + (obj.alpha*err);
                obj.dx = obj.dx + (obj.beta*(err/obj.dt));
                obj.ddx = obj.ddx + (obj.gamma*(err/(0.5*obj.dt^2)));
            end          

            obj.true_ppm = [obj.true_ppm trueValue];
            obj.filtered_ppm = [obj.filtered_ppm obj.x];
            obj.noisy_ppm = [obj.noisy_ppm noisyMeas];
            obj.predicted = [obj.predicted x_est];
            meas = obj.x;
        end

        function obj = reset(obj)
            obj.history.noisy_ppm = [obj.history.noisy_ppm, obj.noisy_ppm];
            obj.history.filtered_ppm = [obj.history.filtered_ppm, obj.filtered_ppm];
            obj.history.true_ppm = [obj.history.true_ppm, obj.true_ppm];
            obj.history.predicted = [obj.history.predicted, obj.predicted];   
            
            obj.noisy_ppm = [];
            obj.filtered_ppm = [];
            obj.true_ppm = [];
            obj.predicted = [];

            obj.x = 0;
            obj.dx = 0;
            obj.ddx = 0;
        end

%% Plot the measurements graph
        function plotF(obj)
            figure
            xa = uint32(1):uint32(size(obj.true_ppm,2) + size(obj.history.true_ppm,2));
            
            hold on
            y1 = [obj.history.true_ppm obj.true_ppm];
            plot(xa,y1,'-g')

            y2 = [obj.history.noisy_ppm obj.noisy_ppm];
            plot(xa,y2,'-r')
            
            y3 = [obj.history.predicted obj.predicted];
            plot(xa,y3,'-y')

            y4 = [obj.history.filtered_ppm obj.filtered_ppm];
            plot(xa,y4,'-b')

            legend('True Values','Noisy Measurements','Predicted Values','Filtered Values')
            hold off
        end
    end
end
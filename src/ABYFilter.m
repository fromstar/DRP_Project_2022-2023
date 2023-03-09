classdef ABYFilter
    properties
        alpha;
        beta;
        gamma;
        
        x
        P

        noisy_ppm
        filtered_ppm
        true_ppm
        predicted

        history
    end

    methods
        function obj = ABYFilter()


            obj.alpha = 0.2;                  
            obj.beta = 0.3;
            obj.gamma = 0.5;

            obj.x = 0; % system estimation
            obj.P = 1; % estimation's variance

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
            if(obj.x == 0)
                obj.x = noisyMeas;
            end
            x_prior = obj.x;
            P_prior = obj.P + obj.gamma;
            e = noisyMeas - x_prior;
            obj.x = x_prior + obj.alpha * e;
            obj.P = (1 - obj.alpha)^2 * P_prior + obj.beta * e^2;
            
            obj.true_ppm = [obj.true_ppm trueValue];
            obj.filtered_ppm = [obj.filtered_ppm obj.x];
            obj.noisy_ppm = [obj.noisy_ppm noisyMeas];
            obj.predicted = [obj.predicted x_prior];
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
            obj.P = 1;
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
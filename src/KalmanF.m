classdef KalmanF
    properties
        A; % State Transition Matrix
        R; % Covariance Noise of the Measurement
        P; % Estimate covariance

        x; % Estimate of the system state
        
        noisy_ppm
        filtered_ppm
        true_ppm
        gain

        history
    end

    methods
        function obj = KalmanF()
            obj.A = [1 0; 0 1];
            obj.R = 10;
            obj.x = [0; 0];
            obj.P = 150;
            
            obj.filtered_ppm = [];
            obj.true_ppm = [];
            obj.noisy_ppm = [];
            obj.gain = [];

            obj.history.noisy_ppm = [];
            obj.history.filtered_ppm = [];
            obj.history.true_ppm = [];
        end
        function [obj,filtered_value] = updateF(obj, noisyMeas, trueValue)
            if(obj.x(1) == 0)
                obj.x(1) = noisyMeas;
            end
            % Predict
            x_pred = obj.A*obj.x;
            P_pred = obj.P;

            % Correction
            % Kalman Gain
            K = P_pred  / (P_pred + obj.R);

            obj.x = x_pred + K * (noisyMeas - x_pred);
            obj.P = (1 - K) * P_pred;
            
            obj.gain = [obj.gain, K];
            obj.noisy_ppm = [obj.noisy_ppm, noisyMeas];
            obj.filtered_ppm = [obj.filtered_ppm, obj.x(1)];
            obj.true_ppm = [obj.true_ppm, trueValue];

            filtered_value = obj.x(1);
        end

        function obj = resetF(obj)
            obj.history.noisy_ppm = [obj.history.noisy_ppm, obj.noisy_ppm];
            obj.history.filtered_ppm = [obj.history.filtered_ppm, obj.filtered_ppm];
            obj.history.true_ppm = [obj.history.true_ppm, obj.true_ppm];

            obj.noisy_ppm = [];
            obj.filtered_ppm = [];
            obj.true_ppm = [];
            obj.x = [0; 0];
            obj.P = 150;
        end
        function plotF(obj,id)
            figure
            xa = uint32(1):uint32(size(obj.true_ppm,2) + size(obj.history.true_ppm,2));
            
            hold on
            y1 = [obj.history.true_ppm obj.true_ppm];
            plot(xa,y1,'-g')

            y2 = [obj.history.noisy_ppm obj.noisy_ppm];
            plot(xa,y2,'-r')

            y3 = [obj.history.filtered_ppm obj.filtered_ppm];
            plot(xa,y3,'-b')

            legend('True Values','Noisy Measurements','Filtered Values')
            hold off
            savefig("img/"+id+"/"+id+"_Kalman_filter.fig")
        end
        function plotG(obj,id)
            x = uint32(1):uint32(size(obj.gain,2));
            y = obj.gain;
            figure
            hold on
                plot(x,y,'-b');
                legend('Kalman Gain')
            hold off
            savefig("img/"+id+"/"+id+"_Kalman_gain.fig")
        end
    end
end
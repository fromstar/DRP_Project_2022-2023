classdef KalmanF
    properties
        A; % State Transition Matrix
        H; % Measure Matrix
        Q; % Covariance Noise
        R; % Covariance Noise Matrix of the Measurement
        
        x; % Estimate of the system state
        P; % Estimate covariance
        
        noisy_ppm
        filtered_ppm
        true_ppm

        history
    end

    methods
        function obj = KalmanF()
            obj.A = [1 1; 0 1];
            obj.H = [1 0];
            obj.Q = [0.01 0; 0 0.01];
            obj.R = 10;
            obj.x = [0; 0];
            obj.P = [1 0; 0 1];
            
            obj.filtered_ppm = [];
            obj.true_ppm = [];
            obj.noisy_ppm = [];

            obj.history.noisy_ppm = [];
            obj.history.filtered_ppm = [];
            obj.history.true_ppm = [];
        end
        function obj = updateF(obj, noisyMeas, trueValue)
            if(obj.x(1) == 0)
                obj.x(1) = noisyMeas;
            end
            % Predict
            x_pred = obj.A*obj.x;
            P_pred = obj.A * obj.P * obj.A' + obj.Q;

            % Correction
            K = P_pred * obj.H' / (obj.H * P_pred * obj.H' + obj.R);
            obj.x = x_pred + K * (noisyMeas - obj.H * x_pred);
            obj.P = (eye(2) - K * obj.H) * P_pred;

            obj.noisy_ppm = [obj.noisy_ppm, noisyMeas];
            obj.filtered_ppm = [obj.filtered_ppm, obj.x(1)];
            obj.true_ppm = [obj.true_ppm, trueValue];
        end

        function obj = resetF(obj)
            obj.history.noisy_ppm = [obj.history.noisy_ppm, obj.noisy_ppm];
            obj.history.filtered_ppm = [obj.history.filtered_ppm, obj.filtered_ppm];
            obj.history.true_ppm = [obj.history.true_ppm, obj.true_ppm];

            obj.noisy_ppm = [];
            obj.filtered_ppm = [];
            obj.true_ppm = [];
            obj.x = [0; 0];
            obj.P = [1 0; 0 1];
        end
        function plotF(obj)
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
        end
    end
end
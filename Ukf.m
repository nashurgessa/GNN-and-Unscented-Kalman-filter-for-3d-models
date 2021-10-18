classdef Ukf < handle % matlab.System
    properties
        Xpred, prediction, S;
        P;
        z;
        x_k, z_k;
        
        my_id;
    end

    % Pre-computed constants
    properties(Access = private)
        intialized=false;
        scale;
        Wm, Wc;
        F, G, Q, H, R, a;
        
        dt, m;
    end

    methods
        % Constructor
        function obj = Ukf(config, data)
            % Support name-value pair arguments when constructing object
            % obj.intialized=false;
            obj.scale = config.SCALE;
            obj.Wm = config.WEIGHTS;
            obj.Wc = config.WEIGHTS;
            obj.F = config.F;
            obj.Q = config.Q;
            obj.H = config.H;
            obj.R = config.R;
            obj.G = config.G;
            obj.S = obj.R;
            obj.a = config.a; % acceleration
            
            obj.dt = config.dt;
            obj.m = config.M;
            
            % Initialize
            obj.intialized = true;
            x = data(:, 1);
            % obj.z = data(1:3, 2);
            L = numel(x);
            obj.Xpred = x;
            obj.P = eye(L, L);
            
            obj.prediction = obj.F * obj.Xpred;
            
            %
            obj.x_k = x;
            obj.z_k = data(1:3, 2);
        end
    end

    methods(Access = public)
        %% Common functions
        function intialize(obj, data)
            obj.intialized = true;
            x = data(:, 1);
            obj.z = data(1:3, 2);
            L = numel(x);
            obj.Xpred = x;
            obj.P = eye(L, L);
        end
        
        function [X] = sigmas(obj, x_, P_, c)
            A = c * chol(P_)';
            Y = x_(:, ones(1, numel(x_)));
            X = [x_ Y+A Y-A];
        end
        
        function [Y] = transitionFn(obj, X_k)
            column_wise = num2cell(transpose(X_k));
            [px, py, pz, vx, vy, vz] = column_wise{:};
            prevState = [px; py; pz; vx; vy; vz];
            Y = obj.F*prevState + obj.G*obj.a;
        end
        
        function [Y] = measurementFn(obj, X_k)
            column_wise = num2cell(transpose(X_k));
            [px, py, pz, vx, vy, vz] = column_wise{:};
            Rr = sqrt(px^2 + py^2 + pz^2);
            thetha=atan(py/px);
            if((px >=0 & py>=0) | (px>=0 & py<0))
                thetha=thetha;
            else
                thetha=thetha+pi;
            end
            phi=atan(pz/sqrt(px^2+py^2));
            
            if ((px^2+py^2)^2) < 0.0001
                Y = [0.0; 0.0; 0.0];
            else
                Y = [Rr; thetha; phi];
            end
        end
       
        function [y, Y, P, Y1] = ut(obj, f, X, Wm, Wc, n, R)
            L = size(X, 2);
            y = zeros(n, 1);
            Y = zeros(n, L);
            for k=1:L
                if f
                    Y(:, k) = obj.transitionFn(X(:, k));
                else
                    Y(:, k) = obj.measurementFn(X(:, k));
                end
                y = y+Wm(k)*Y(:,k);
            end
            Y1 = Y-y(:,ones(1,L));  
            P = Y1*diag(Wc)*Y1'+R;
        end
        
        function ukf(obj, x_pred, P_k, z_k)
            L = numel(x_pred);
            m = numel(z_k);
            [X] = obj.sigmas(x_pred, P_k, obj.scale);
            fstate = true;
            hmeas = ~fstate;
            [x1, X1, P1, X2] = obj.ut(fstate, X, obj.Wm, obj.Wc, L, 0.00001*obj.Q);
            [z1, Z1, P2, Z2] = obj.ut(hmeas, X1, obj.Wm, obj.Wc, m, obj.R);
            
            % Obj.s
            obj.S = P2;
            
            % Update
            P12 = X2*diag(obj.Wc)*Z2';                          
            K = P12*inv(P2);  
            obj.Xpred = x1+K*(z_k-z1);                                
            obj.P = P1-K*P12'; 
            
        end
        
        function update(obj, data)
            % data = squeeze(data);
%             if ~obj.intialized
%                 obj.intialize(data);
%             else
%                 % current value of measurment
%                 obj.z = data(1:3, 2);
%                 obj.Xpred = data(:, 1);
%                 obj.ukf(obj.Xpred, obj.P, obj.z);
%             end
              obj.z = data(1:3, 2);
              obj.Xpred = data(:, 1);
              obj.ukf(obj.Xpred, obj.P, obj.z);
              
              % update 
              obj.prediction = obj.F * obj.Xpred;
              
              % Store input data
              obj.x_k = data(:, 1);
              obj.z_k = data(1:3, 2);
        end
        
        
        function [likelihoods] = likelihood(obj, measurement)
            predictions = obj.H * obj.prediction; % yk = Hz + vk, R
            continousPrediction = predictions - obj.H * obj.Xpred; % yk - Hz_1
            timeShiftedPrediction = obj.H * obj.Xpred + (obj.dt * continousPrediction);
            
            likelihoods = normalDistributionDensity(obj.S, timeShiftedPrediction, measurement, obj.m);
        end
        
        function [x_pred] = getPredictedState(obj)
            x_pred = obj.Xpred;
        end
    end
end

classdef ParameterSetting < handle
    
    properties (Access = public)
        NX, NAUGMENTED, NSIGMA, M;
        STD_SPEED_NOISE, STD_YAWRATE_NOISE, VAR_SPEED_NOISE, VAR_YAWRATE_NOISE;
        
        SCALE; 
        WEIGHTS;
        
        NZ_RADAR, VAR_RHO, VAR_PHI, VAR_RHODOT;
        F, G, Q, H, R, a;
        dt;
    end
    
    methods 
        function this = ParameterSetting()
            % number of state
            this.NX = 6;
            % number of states plus two noise values
            this.NAUGMENTED = 6;  % L
            % number of sigma points
            this.NSIGMA =this.NAUGMENTED * 2 +1;  % 
            
            % length of measurement
            this.M = 3;
            
            % timestamp
            this.dt = 1;

            % process noise standard deviation
            % longitudinal acceleration in m/s^2
            % this.STD_SPEED_NOISE = 0.9;

            % yaw acceleration in rad/s^2
            % this.STD_YAWRATE_NOISE = 0.6;

            % this.VAR_SPEED_NOISE = this.STD_SPEED_NOISE * this.STD_SPEED_NOISE;
            %  this.VAR_YAWRATE_NOISE = this.STD_YAWRATE_NOISE * this.STD_YAWRATE_NOISE;

            %% 
            a = 1e-3;
            beta=3;
            ki=0;
            LAMBDA = a^2 * (this.NAUGMENTED + ki) - this.NAUGMENTED;
            % LAMBDA = 3 - this.NAUGMENTED; % parameter for tuning
            this.SCALE = sqrt(LAMBDA + this.NAUGMENTED);  % used to create augmented sigma


            W = 0.5/ (LAMBDA + this.NAUGMENTED);
            W_0 = LAMBDA / (LAMBDA + this.NAUGMENTED);
            this.WEIGHTS = [W_0, W, W, W, W, W, W, W, W, W, W, W, W];
%             this.WEIGHTS = [W_0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 
%                              0, W; W; W; W; W; W; W; W; W; W; W; W;];

            %%
            % 
            this.NZ_RADAR = 3;
            
            % RADAR measurements noise standard deviations
            STD_RHO = 2.3;  % METERS
            STD_PHI = 0.03; % radians
            STD_RHODOT = 0.001;  % meters/second
            this.VAR_RHO = STD_RHO * STD_RHO;
            this.VAR_PHI = STD_PHI * STD_PHI;
            this.VAR_RHODOT = STD_RHODOT * STD_RHODOT;
                 
            this.F = [1, 0, 0, this.dt, 0, 0;
                      0, 1, 0, 0, this.dt, 0;
                      0, 0, 1, 0, 0, this.dt;
                      0, 0, 0, 1, 0, 0;
                      0, 0, 0, 0, 1, 0;
                      0, 0, 0, 0, 0, 1];
                  
           this.G = [0.5*this.dt^2, 0, 0;
                     0, 0.5*this.dt^2, 0;
                     0, 0, 0.5*this.dt^2;
                     this.dt, 0, 0;
                     0, this.dt, 0;
                     0, 0, this.dt];
           ax = 0.2;
           ay = 0.01;
           az = 0.01;
           this.a = [ax; ay; az];
                  
           var_x = 0.01;
           var_y = 0.01;
           var_z = 0.01;
           var_vx = 0.01;
           var_vy = 0.01;
           var_vz = 0.01;
           this.Q =[var_x^2, 0, 0, var_x*var_vx, 0, 0;
                    0, var_y^2, 0, 0, var_y*var_vy, 0;
                    0, 0, var_z^2, 0, 0, var_z*var_vz;
                    var_x*var_vx, 0, 0, var_vx*var_vx, 0, 0;
                    var_y*var_vy, 0, 0, var_vy*var_vy, 0, 0;
                    var_z*var_vz, 0, 0, var_vz*var_vz, 0, 0;];
            
           this.H = [1, 0, 0, 0, 0, 0;
                     0, 1, 0, 0, 0, 0;
                     0, 0, 1, 0, 0, 0];
                 
           this.R = [this.VAR_RHO, 0, 0;
                     0, this.VAR_PHI, 0;
                     0, 0, this.VAR_RHODOT];
                  
        end
    end
        
    
end
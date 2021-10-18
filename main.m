%%
clear all;
close;
clc;

% Number of data point
N = 2400;
% time difference dt = 1
T = 1;

%%
% Input Line 1
Rx1=2500; Ry1=600; Rz1=0;
vx1=5; vy1=0; vz1=0;
ax1=0; ay1=0; az1=0;

%%

%%
% Input Line 2
Rx2=-10000; Ry2=-10000; Rz2=0;
vx2=100; vy2=10; vz2=0;
ax2=0; ay2=0; az2=0;
%%

%%
%Input circle 1
Rx3=100000; Ry3=20000;Rz3=100;
v3=100; w3=0.005;  % vx = 2000, make sure to use or not this one
%%

%%
% Polar and rectangular generator
[polar_vector1, rect_vector1] = fun_gen_line(Rx1, Ry1, Rz1, vx1, vy1,...
     vz1, ax1, ay1, az1, N, T);
 
[polar_vector2, rect_vector2] = fun_gen_line(Rx2, Ry2, Rz2, vx2, vy2,...
     vz2, ax2, ay2, az2, N, T );
 
[polar_vector3, rect_vector3] = fun_gen_circle( Rx3,Ry3,Rz3,v3,w3,N,T);
%%

rng('default');
noise_x = randn(1, N) * 50;
noise_y = randn(1, N) * 50;
noise_z = randn(1, N) * 50;
zeros_h = zeros(1, N);
noise_xyz = [noise_x; noise_y; noise_z; zeros_h; zeros_h; zeros_h];
matrix_line_noise_1 = rect_vector1 + noise_xyz;
matrix_line_noise_2 = rect_vector2 + noise_xyz;
matrix_line_noise_3 = rect_vector3 + noise_xyz;


sigma_r=5 ; % rho noise
sigma_t=0.0384; % thetha noise
sigma_p=0.0; % zhetha noise
noise_r = randn(1, N) * sigma_r;
noise_t = randn(1, N) * sigma_t;
noise_p = randn(1, N) * sigma_p;
noise_polar = [noise_r; noise_t; noise_p; zeros_h; zeros_h; zeros_h];
polar_coordinate_noise_1 = polar_vector1 + noise_polar; 
polar_coordinate_noise_2 = polar_vector2 + noise_polar;
polar_coordinate_noise_3 = polar_vector3 + noise_polar;

%%
config = ParameterSetting();
config.dt = T;
%%

%%
% Initializing tracker
% tracker = tracking(config);
tracker = tracking(config);
%%

%%
% number of target
NT = 1;
rmse = zeros(NT, N);
polar_truth_stores = zeros(NT, 3, N);
polar_noise_stores = zeros(NT, 3, N);
predictions_stores = zeros(NT, 6, N);
predicted_polar_store = zeros(NT, 3, N);
rms = 0;

for k = 1:N
    fprintf("Iteration = %d\n",k);
    
    sensor_data = [matrix_line_noise_1(:, k), polar_coordinate_noise_1(:, k), rect_vector1(:, k), polar_vector1(:, k)];
    measurements(1, :, :) = sensor_data;
    
    sensor_data = [matrix_line_noise_2(:, k), polar_coordinate_noise_2(:, k), rect_vector2(:, k), polar_vector2(:, k)];
    measurements(2, :, :) = sensor_data;
 
    sensor_data = [matrix_line_noise_3(:, k), polar_coordinate_noise_3(:, k), rect_vector3(:, k), polar_vector3(:, k)];
    measurements(3, :, :) = sensor_data;
    
    tracker.filters = tracker.track(measurements)
    
    for i = 1: length(tracker.filters)
        filter = tracker.filters(3, 1);
        [x_pred] = filter.getPredictedState();
        pred_polar = xyzcord2polar(x_pred, T);
        predicted_polar_store(1, 1:3, k) = pred_polar;
        predicted_polar_store_ = squeeze(predicted_polar_store);
        
        figure(1)
        polarplot(polar_coordinate_noise_3(2, 1:k), polar_coordinate_noise_3(1, 1:k), 'g');  %1:k
        polarplot(polar_vector3(2, 1:k), polar_vector3(1, 1:k), 'b');
        polarplot(predicted_polar_store_(2, 1:k), predicted_polar_store_(1, 1:k), 'r');
        hold on;
    end
    

end

% [x_pred] = ukf.getPredictedState();
% pred_polar = xyzcord2polar(x_pred, T);
% predicted_polar_store(1, 1:3, k) = pred_polar;
% predicted_polar_store_ = squeeze(predicted_polar_store);

% figure(1)
% polarplot(polar_coordinate_noise_3(2, 1:k), polar_coordinate_noise_3(1, 1:k), 'g');  %1:k
% polarplot(polar_vector3(2, 1:k), polar_vector3(1, 1:k), 'b');
% polarplot(predicted_polar_store_(2, 1:k), predicted_polar_store_(1, 1:k), 'r');
% hold on;

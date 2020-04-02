%% RVC example
% Example file to drive a robot with a constant speed and steering angle,
% record the measured and the true angle and plot the results

%% Initialization
clear all;
close all;

startup_rvc

% Parameters
v = 1.0;    % forward velocity
gamma = 0.3;    % steer angle
n = 100; % Number of iterations to simulate
dt = 0.1; % Timsteps

% Vehicle object
X = [0; 0; 0];
Q=diag([0.05,0.2].^2);  % noise on travelled distance[m] and the heading angle [rad]
P = diag([0.5, 0.5, 0.5].^2);
vehicle = Bicycle(Q,'x0',X, 'accelmax',1, 'speedmax',1, 'steermax',1, 'dt',dt);
vehicle.V_IMU = 0.05^2; % Noise on gyroscope measurements

% Initialize collections
time = zeros(1,n); % Timestamps
theta_gyro = zeros(1,n); % Measurements returned by gyroscope
theta_robot = zeros(1,n); % Actual angle of robot

% Initialize figure
figure
xlim([-20,20])
ylim([-20,20])

%% Run simulation
for i=2:n 
    odom = vehicle.step(v, gamma); % Simulate 1 timestep and return odometry
    
    theta_gyro(i) = vehicle.get_IMU(); % Get measured robot orientation
    theta_robot(i) = vehicle.x(3);     % Get real orienatation
    time(i) = time(i-1) + dt;          % Get timestep
    
    % Plot results and prepare for next loop
    vehicle.plot();
end

%% Plotting

% Individual coordinates
figure()
plot(time, theta_gyro)
hold on
plot(time, theta_robot)
xlabel('Time [s]')
ylabel('Angle [rad]')
legend('Measured angle', 'True angle')

% True trajectory
figure()
vehicle.plot_xy()
xlim([-10,10])
ylim([-10,10])
title('Trajectory')
legend('Followed path')

%% RVC example 2
% Drive robot along a random path, return measurments to landmarks and
% visualize them

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
% Map object
map = LandmarkMap(20,20);

% Sensor object
W = diag([0.01, deg2rad(1)].^2) % Uncertainty on range and bearing measureùents
sensor = RangeBearingSensor(vehicle, map, W);

% Driver object
driver = RandomPath(10);
vehicle.add_driver(driver);

% Initialize figure
figure
map.plot()
hold on
xlim([-20,20])
ylim([-20,20])

%% Run simulation
for i=i:n
    odo = vehicle.step(); % Simulate 1 timestep and return odometry
   
    % Get measurement
    [z, identity] = sensor.reading();

    % Plot results
    vehicle.plot()
    sensor.plot(identity);

end
%% INTRODUCTION
%   To start using a turtlebot 3 following procedure
%   needs to be followed:
%
%   1) Turn the TB3 on.
%   2) Set up the putty SSH connection (IP adress is given)
%   3) Type "bringup".
%   4) You are now able to run your MATLAB code
%
% This file file load in a map and localize the robot by fitting current
% laser scan measurements to this map with a particle filter (PF). 
% The results are fused with odometry (dead reckoning) by a Kalman filter.
% All results are plotted at the end, whilst driving, particle filter
% results are also shown.
%% INITIALIZATION
close all
clear all
clc

ip = '192.168.42.1';    % This needs to be the ip adres of the ROS master
turtlebot = Turtlebot_GT(ip); % Initialize turtlebot communication

t_test = 15; % test duration, robot will drive forwards for this amount of seconds
% Parameters for kalman filter
Pcov_EKF = eye(3)*0.5^2; % Initial covariance
R = eye(3)*0.05^2; % Measurement noise

% Load map
filePath = fullfile(fileparts(which('GT_mobile_robotics')),'map.mat');
load(filePath);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% DONT CHANGE ANYTHING AFTER THIS LINE %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

C = eye(3);
Pcov_DR = Pcov_EKF;

pose_EKF = [5;5;0];
pose_PF = pose_EKF;
pose_DR = pose_EKF;

Pcov_matrix = [];
pose_DR_matrix = [];
pose_PF_matrix = [];
pose_EKF_matrix = [];
S_matrix = [];
thetax_matrix = [];

% Parameters for particle filter
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.10 3.5];
rangeFinderModel.Map = map;
rangeFinderModel.NumBeams = 360;
amcl = robotics.MonteCarloLocalization;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
amcl.ParticleLimits = [10 50];
amcl.GlobalLocalization = false;
amcl.InitialPose = pose_EKF;
amcl.InitialCovariance = eye(3)*0.5;
visualizationHelper = ExampleHelperAMCLVisualization(map);
i = 0;

turtlebot.set_linear_angular_velocity(0.1,0.0)
%% MAIN LOOP
tic
while toc <= t_test
    %% PREDICT
    % Receive laser scan and odometry DATA
    [ds,dth] = turtlebot.get_odometry();
    scan = turtlebot.get_scan();
    
    % Predict new pose with motion model
    [Pcov_EKF, pose_EKF] = motionModel(pose_EKF, Pcov_EKF, ds, dth, 1);
    [Pcov_DR, pose_DR] = motionModel(pose_DR, Pcov_DR, ds, dth, 1);
    pose_DR_matrix(1:3,end+1) = pose_DR ;
    
    %% PARTICLE FILTER UPDATE
    % Update estimated robot's pose and covariance using new odometry and sensor readings.
    [isUpdated,pose_PF, estimatedCovariance] = amcl(pose_DR, scan.Ranges, scan.Angles);
    pose_PF = pose_PF';
    pose_PF_matrix(1:3,end+1) = pose_PF;
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, pose_PF, scan.Ranges, scan.Angles, i);
    end
    
    %% KALMAN FILTER FUSION
    R = R;
    v = pose_PF - pose_EKF;
    K = Pcov_EKF*C'*inv(C*Pcov_EKF*C' + R);
    pose_EKF = pose_EKF + K*v;
    Pcov_EKF = (eye(3)-K*C)*Pcov_EKF;
    
    % Singular value decomposition
    [S,normV] = singularValue(Pcov_EKF); % Singular value decomposition for error ellipse
    thetax = acos(dot([1,0],normV(:,1))/(norm([1,0])*norm(normV(:,1)))); % Angle of tilted ellipse
    
    % Save results for plot
    Pcov_matrix(1:3,end+1:end+3) = Pcov_EKF;
    pose_EKF_matrix(1:3,end+1) = pose_EKF;
    S_matrix(1:2,end+1:end+2) = S;
    thetax_matrix(end+1) = thetax;
    
end
turtlebot.stop
rosshutdown

%% PLOTTING
figure('Name', 'Robot path')
show(map)
hold on
uncertainty_ellipse = zeros(2,size(pose_EKF_matrix,2)*100);

for i=1:5:size(pose_EKF_matrix,2)
    pose = pose_EKF_matrix(1:3,i);
    S = S_matrix(1:2,2*i-1:2*i);
    thetax = thetax_matrix(i);
    [xE,yE] = plotEllipse(2*sqrt(S(1,1)),2*sqrt(S(2,2)),pose(1),pose(2),thetax);
    uncertainty_ellipse(1,100*i-99:100*i) = xE;
    uncertainty_ellipse(2,100*i-99:100*i) = yE;
end

plot(pose_DR_matrix(1,:),pose_DR_matrix(2,:),'r','DisplayName','DR path')
hold on
plot(pose_PF_matrix(1,:),pose_PF_matrix(2,:),'g','DisplayName','PF path')
plot(pose_EKF_matrix(1,:),pose_EKF_matrix(2,:),'b','DisplayName','EKF path')
plot(uncertainty_ellipse(1,:), uncertainty_ellipse(2,:),'black.','DisplayName','Uncertainty')
legend('show')
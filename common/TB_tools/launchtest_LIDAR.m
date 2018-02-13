%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Template Routine to see if the Lidar is in good shape %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cleanup MATLAB
close all
clear all
clc

% Initialize Turtlebot
ip = '192.168.42.1';            % IP addres of the turlebot
turtlebot = Turtlebot_GT(ip);   % Initialize Turtlebot object
t_test = 5;                     % Duration of the LIDAR test
tic;                            % Start stopwatch

while toc < t_test             
    figure(1)
    scan = turtlebot.get_scan()                 % Get LIDAR scan data
    [x,y]=pol2cart(scan.Angles,scan.Ranges);    % Convert to Carthesian coordinates
    scatter(x,y);                                % Plot coordinates
    axis(5*[-1,1,-1,1]);
    grid on
end

rosshutdown;                   % Shutdown ROS master

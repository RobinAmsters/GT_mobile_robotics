%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Template Routine to see if the robot is in good shape %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cleanup MATLAB
close all
clear all
clc

% Initialize Turtlebot
ip = '192.168.42.1';            % IP addres of the turlebot
turtlebot = Turtlebot_GT(ip);   % Initialize Turtlebot object
t_test = 2;                     % Duration of the test

turtlebot.set_linear_angular_velocity(0.1,0); % Start driving
pause(t_test);                  % Wait for a number of seconds
turtlebot.stop()                % Stop driving

rosshutdown;                    % Shutdown ROS master

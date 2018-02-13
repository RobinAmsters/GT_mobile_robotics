clear all
close all
clc
%% INTRODUCTION
%   To start using a turtlebot 3 following procedure
%   needs to be followed:
%
%   1) Turn the TB3 on.
%   2) Set up the putty SSH connection (IP adress is given)
%   3) Type "bringup".
%   4) You are now able to run your MATLAB code
%
%% Excercise 1_1_1
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Connection to the ROS node system
ip = '192.168.42.1';    % This needs to be the ip adres of the ROS master
turtlebot = Turtlebot_GT(ip); % Make Turtlebot object
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

s=0; % Start distance is 0m
???  % Find the function to make the robot start driving

while abs(s) < dist       
    [ds,dth] = ??? % Check the distance the robot has driven
    s = s + ds;    % Add displacement to the total distance
end

??? % Find the function to make the robot stop
rosshutdown; % Shutdown ROS master
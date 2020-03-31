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
% This file file will let the turtlebot drive a square pattern. Whilst
% driving, it will build an occupancy grid map. This map is saved and can
% later be used for localization.

%% MAPPING
% Connection to the ROS node system
ip = '192.168.42.1';    % This needs to be the ip adres of the ROS master
turtlebot = Turtlebot_GT(ip); % Make Turtlebot object

% initialize map
map = robotics.OccupancyGrid(15,15,20);
pose = [5;5;0];
Pcov = 0.01*eye(3);

% drive square and map environment
dist = 0.25;
angle = pi/2;
speed = 0.1;
for i=1:4
    [map, pose, Pcov] = map_while_driving(turtlebot, map, dist, speed, pose, Pcov);
    [map, pose, Pcov] = map_while_turning(turtlebot, map, angle, -speed*2, pose, Pcov);
end

turtlebot.stop() % stop driving
save 'map.mat' map % save map
rosshutdown % shutdown ros master

%% FUNCTION DEFINITIONS
function [map, pose, Pcov] = map_while_driving(turtlebot, map, dist, speed, pose, Pcov)
turtlebot.set_linear_angular_velocity(speed,0.0);
d = 0;
maxrange = 5;
while abs(d) <= dist
    % Receive laser scan and odometry DATA
    [ds,dth] = turtlebot.get_odometry();
    
    % Predict new pose with motion model
    [~, pose] = motionModel(pose, Pcov, ds, dth, 1);
    d = d + ds;
    
    % Get scan data
    scan = turtlebot.get_scan();
    
    % Add to map
    insertRay(map,pose,scan.Ranges,scan.Angles,maxrange);
    show(map)
end
end

function [map, pose, Pcov] = map_while_turning(turtlebot, map, angle, speed, pose, Pcov)
turtlebot.set_linear_angular_velocity(0.0,speed);
maxrange = 5;
th = 0;
while abs(th) <= angle
    % Receive laser scan and odometry DATA
    [ds,dth] = turtlebot.get_odometry();
    
    % Predict new pose with motion model
    [~, pose] = motionModel(pose, Pcov, ds, dth, 1);
    th = th + dth;
    
    % Get scan data
    scan = turtlebot.get_scan();
    
    % Add to map
    insertRay(map,pose,scan.Ranges,scan.Angles,maxrange);
    show(map)
end
end


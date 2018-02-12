function stop_robot()
% Function that makes the robot stop driving (sends a velocity message with
% all velocities equal to zero.
%
% INPUTS:
%   - None
%
% OUTPUTS:
% - None
%
% Robin Amsters, 2017

topicname = '/cmd_vel';                     % Topic to send velocity commands to Robot
msgtype = 'geometry_msgs/Twist';            % The message type for the velocity
vel_pub = rospublisher(topicname,msgtype);  % ROS publisher for sending information
vel_msg = rosmessage(vel_pub);              % Message object

% Set linear velocities to 0
vel_msg.Linear.X = 0.0;
vel_msg.Linear.Y = 0.0;
vel_msg.Linear.Z = 0.0;

% Set angular velocities to 0
vel_msg.Angular.X = 0;
vel_msg.Angular.Y = 0;
vel_msg.Angular.Z = 0;

% Publish velocity message
send(vel_pub,vel_msg);  
end
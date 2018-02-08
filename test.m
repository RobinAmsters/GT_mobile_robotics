%% TESTING FILE

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Connection to the ROS node system
 ip = '192.168.42.1';    % This needs to be the ip adres of the ROS master
turtlebot = Turtlebot_GT(ip); % Make Turtlebot object

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 V = turtlebot.get_battery_voltage()
%   path = fileparts(mfilename('fullpath') );
%   folder_path = fullfile(path, 'common')
%   rosgenmsg(folder_path)

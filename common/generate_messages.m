% Generate the MATLAB files neccesary for processing costum ROS messages.

path = fileparts(mfilename('fullpath') ); 
folder_path = fullfile(path) 
rosgenmsg(folder_path)

% Run the next line seperately 
%rosmsg('show','turtlebot3_msgs/SensorState')
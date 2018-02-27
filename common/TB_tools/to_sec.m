function [t_sec] = to_sec(t_ros_msg)
% Convert a ROS timestamp to UNIX time as a double.
%
% INPUTS:
%   - t_ros_msg = timestamp expressed in a msg.Stamp format/
% OUTPUTS:
%   - t_sec = UNIX time expressed as a double

t_sec = double(t_ros_msg.Sec)+double(t_ros_msg.Nsec)*10^-9;
end
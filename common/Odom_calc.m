function [s,s_ang] = Odom_calc(x,y,ang,ds,first_time,s_set_x,s_set_y,s_set_ang)
%
    if (first_time < 0.2)               % Set the origin of axis in starting point
        ds_x = ds.Position(1) - s_set_x;% Define x-coordinate
        ds_y = ds.Position(2) - s_set_y;% Define y-coordinate
        ds_ang = ds.Orientation(1) -s_set_ang;
    else
        ds_x = ds.Position(1) - x;      % Define x-coordinate
        ds_y = ds.Position(2) - y;      % Define y-coordinate
        ds_ang = ds.Orientation(1) - ang;
    end
    x = ds.Position(1);
    y = ds.Position(2);
    ang = ds.Orientation(1);
    %
    ds = sqrt((ds_x^2)+(ds_y^2));       % Real travelled distance
    %                         
    s = s + ds                          % This ds is the travveled distance from the starting point
    s_ang = s_ang + abs(ds_ang)
    if (s_ang > 6)
        s_ang = abs(2*pi - s_ang)
    end
end
% Costum turtlebot class definition for use in GT mobile robotics education
%
% Properties:
%
% Functions:
%
% Robin Amsters - 2017

classdef Turtlebot_GT
    properties
        ip           % IP address of the robot. By default this should be 192.168.42.1
        turtlebot    % turtlebot object from the MATLAB ROS toolbox
        vel_pub      % publisher object for the /cmd_vel node
        vel_msg      % Twist message that gets published by the velocity publisher
        s_set        % Defining starting coordinates
    end
    methods
        function turtle = Turtlebot_GT(ip)
            turtle.ip = ip;
            rosshutdown;                                                      % To make sure there are no MATLAB nodes still running
            rosinit(ip);                                                      % Setting the connection between MATLAB and ROS
            turtle.turtlebot = turtlebot(ip);                                 % Initialize turtlebot object (depends on turtlebot support package)
            turtle.vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist'); % ROS publisher for velocity commands
            turtle.vel_msg = rosmessage(turtle.vel_pub);
            turtle.s_set = getOdometry(turtle.turtlebot);
        end
        
        function [s, s_ang] = odom_calc(turtle)
            ds = getOdometry(turtle.turtlebot);        % check the current location
            s_set_x = turtle.s_set.Position(1);        % Define the necessary data from struct format
            s_set_y = turtle.s_set.Position(2);        % X and Y coordinates are neccesary because
            s_set_ang = turtle.s_set.Orientation(1);   % The angle with respect to x-axis
            [s,s_ang] = Odom_calc(ds,s_set_x,s_set_y,s_set_ang);
        end
        
        function [scan] = get_scan(turtle)
            % Get data from laser scanner
            % 
            % INPUTS: 
            %   - None
            % OUTPUTS:
            %   - scan = Laser scan measurements in a struct with field
            %   'Ranges' [360×1 double] and 'Angles' [360×1 double].
            %
            %   Example: 
            %        scan = turtlebot.get_scan()                 % Get LIDAR scan data
            %        [x,y]=pol2cart(scan.Angles,scan.Ranges);    % Convert to Carthesian coordinates
            %        scatter(x,y);                               % Plot coordinates
            
            scan = getLaserScan(turtle.turtlebot);
        end
        
        function set_linear_angular_velocity(turtle, v,w)
            % Function to apply a linear and angular velocity to the robot
            %
            % INPUTS:
            %   - v = linear velocity along the X axis of the robot [m/s]
            %   - w = angular velocity around the Z axis of the robot [rad/s]
            % OUTPUTS:
            %   - None
            
            % Set linear velocity
            turtle.vel_msg.Linear.X = v;
            % Set angular velocity
            turtle.vel_msg.Angular.Z = w;
            
            % Publish velocity message
            send(turtle.vel_pub,turtle.vel_msg);
        end
        
        function stop(turtle)
            % Set all speeds of the robot to zerp
            %
            % INPUTS:
            %   -None
            % OUTPUTS:
            %   -None
            
            % Set all linear velocities and angular velocities to 0
            turtle.vel_msg.Linear.X = 0.0;
            turtle.vel_msg.Linear.Y = 0.0;
            turtle.vel_msg.Linear.Z = 0.0;
            turtle.vel_msg.Angular.X = 0;
            turtle.vel_msg.Angular.Y = 0;
            turtle.vel_msg.Angular.Z = 0;
            
            % Publish velocity message
            send(turtle.vel_pub,turtle.vel_msg);
        end
    end
end

% Costum turtlebot class definition for use in GT mobile robotics education
%
% Properties:
%     ip: IP address of the robot. By default this should be 192.168.42.1
%     turtlebot: turtlebot object from the MATLAB ROS toolbox You can use 
%                this to access the functions from the Turtlebot Support 
%                Package, but you will rarely need those.
%     vel_pub: publisher object for the /cmd_vel topic
%     vel_msg:  Twist message that gets published by the velocity publisher
%     odom_sub: Subscriber to /odom topic
%     imu_sub: Subscriber to /imu topic
%     odom_prev: Previous position
%     sensor_sub: Subscriber to sensor state
%     timeout: Number of seconds to wait for ros messages
%     R: Wheel radius [m], default = 0.033 m
%     B: Wheelbase [m], default = 0.16 m
%     tick_to_rad: Conversion factor for ticks to radians [rad/tick],
%                  default = 0.001533981 rad/tick
%     w_max: maximum rotational speed [rad/s], default = 2.5 rad/s. 
%            Requesting a higher speed will result in the output being 
%            saturated and a warning being raised.
%     v_max: maximum forward speed [m/s], default = 0.2 m/s. R
%            Requesting a higher speed will result in the output being 
%            saturated and a warning being raised.
%
% Functions:
%   [V_battery] = get_battery_voltage(turtle)
%   [enc_left, enc_right, time] = get_encoder_counts(turtle)
%   [theta] = get_imu(turtle)
%   [v, w, time] = get_linear_angular_velocity(turtle)
%   [ds,dth] = get_odometry(turtle)
%   [scan] = get_scan(turtle)
%   set_wheel_speeds(turtle, W_R, W_L)
%   set_linear_angular_velocity(turtle, v, w)
%   set_linear_velocity_radius(turtle, v, r)
%   drive_straight(turtle, distance, speed)
%   drive_arc(turtle, speed, angle, r)
%   turn_angle(turtle, angle, angular_speed)
%   stop(turtle)
%
% Robin Amsters - 2017-2019

classdef Turtlebot_GT < handle
    properties
        ip           % IP address of the robot. By default this should be 192.168.42.1
        turtlebot    % turtlebot object from the MATLAB ROS toolbox
        vel_pub      % publisher object for the /cmd_vel node
        vel_msg      % Twist message that gets published by the velocity publisher
        odom_sub     % Subscriber to /odom topic
        imu_sub      % Subscriber to /imu topic
        odom_prev    % Previous position
        sensor_sub   % Subscriber to sensor state
        timeout      % Number of seconds to wait for ros messages
        R = 0.033;   % Radius of wheels [m]
        B = 0.16;    % Wheelbase [m]
        tick_to_rad = 0.001533981 % Conversion factor [rad/tick](found in turtlebot3_core_config.h)
        w_max = 2.5 % Maximum rotational velocity of the Turtlebot [rad/s]. When attempting to apply a higher speed, the output will saturate to this value.
        v_max = 0.2 % Maximum forward speed for the Turtlebot [m/s]. When attempting to apply a higher speed, the output will saturate to this value.
    end
    methods
        function turtle = Turtlebot_GT(ip)
            turtle.ip = ip;
            rosshutdown;                                                      % To make sure there are no MATLAB nodes still running
            rosinit(ip);                                                      % Setting the connection between MATLAB and ROS
            turtle.turtlebot = turtlebot(ip);                                 % Initialize turtlebot object (depends on turtlebot support package)
            turtle.vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist'); % ROS publisher for velocity commands
            turtle.vel_msg = rosmessage(turtle.vel_pub);
            turtle.imu_sub = rossubscriber('/imu');  
            turtle.odom_sub = rossubscriber('/odom');
            turtle.odom_prev = getOdometry(turtle.turtlebot);
            turtle.sensor_sub = rossubscriber('/sensor_state');
            turtle.timeout = 1.0; % Let subscribers wait for a maximum number of seconds
        end
        
        function [V_battery] = get_battery_voltage(turtle)
            % Return battery voltage of the Turtlebot
            %
            % Usage: V_battery = turtlebot.get_battery_voltage()
            %
            % INPUTS:
            %   - None
            % OUTPUTS:
            %   - V_battery = battery voltage [V]
            
            sensor_msg = receive(turtle.sensor_sub,turtle.timeout);
            V_battery = sensor_msg.Battery;
        end
        
        function [enc_left, enc_right, time] = get_encoder_counts(turtle)
            % Returns left and right encoder counter values.
            % WARNING: occasionaly this value, which is an 12 bit integer,
            % overflows. Causing very large discontinuities in wheel speed
            % calculations
            %
            % Usage: [enc_left, enc_right, time] = turtlebot.get_encoder_counts()
            %
            % INPUTS:
            %   - None
            % OUTPUTS:
            %   - enc_left = left encoder counter value [tics]
            %   - enc_right = right encoder counter value [tics]
            %   - time = message timestamp converted to UNIX time [s]
            
            sensor_msg = receive(turtle.sensor_sub,turtle.timeout);
            enc_left = sensor_msg.LeftEncoder;
            enc_right = sensor_msg.RightEncoder;
            t_ros_msg = sensor_msg.Header.Stamp;
            time = double(t_ros_msg.Sec)+double(t_ros_msg.Nsec)*10^-9;
        end
        
        function [theta] = get_imu(turtle)
            % Returns the angle as measured by the gyroscope
            %
            % Usage: theta = turtlebot.get_imu()
            %
            % INPUTS:
            %   - None
            % OUTPUTS:
            %   - theta = heading angle [rad]
            
            imu_msg = receive(turtle.imu_sub,turtle.timeout);
            orientation = imu_msg.Orientation;
            quat = [orientation.W, orientation.X, orientation.Y, orientation.Z];
            euler = quat2eul(quat);
            theta = euler(1);
        end
        
        function [v, w, time] = get_linear_angular_velocity(turtle)
            % Return measured linear and angular velocity (based on encoder 
            % tics)
            %
            % Usage: [v, w, time] = turtlebot.get_linear_angular_velocity()
            %
            % INPUTS:
            %   - None
            % OUTPUTS:
            %   - v = forward velocity along direction of turtlebot [m/s]
            %   - w = angular velocity around vertical [rad/s]
            %   - time = message timestamp converted to UNIX time [s]
            
            odom_msg = receive(turtle.odom_sub,turtle.timeout);
            v = odom_msg.Twist.Twist.Linear.X;
            w = odom_msg.Twist.Twist.Angular.Z;
            t_ros_msg = odom_msg.Header.Stamp;
            time = double(t_ros_msg.Sec)+double(t_ros_msg.Nsec)*10^-9;
            
        end
        function [ds,dth,time] = get_odometry(turtle)
            % Return distance driven and angle turned since the last call
            % to this function.
            %
            % Usage: [ds,dth] = turtlebot.get_odometry()
            %
            % INPUTS:
            %   - None
            % OUTPUTS:
            %   - ds = Distance driven since the last function call. This
            %          is defined as: ds = sqrt(dx^2 + dy^2). With dx and
            %          dy the distance traveled along the x- and y-axes,
            %          respectively.
            %   - dth = Angle turned around the vertical axis since the
            %           last function call.
            
            % Get odometry data
            [odom,odom_msg] = getOdometry(turtle.turtlebot);
            
            % Extract relevant coordinates
            dx = odom.Position(1) - turtle.odom_prev.Position(1);
            dy = odom.Position(2) - turtle.odom_prev.Position(2);
            dth = wrapToPi(odom.Orientation(1) - turtle.odom_prev.Orientation(1));
            
            % Conversions
            ds = sqrt(dx^2 + dy^2);
            
            % Save current position for next call
            turtle.odom_prev = odom;
            
            t_ros_msg = odom_msg.Header.Stamp;
            time = double(t_ros_msg.Sec)+double(t_ros_msg.Nsec)*10^-9;
            
        end
        
        function [scan] = get_scan(turtle)
            % Get data from laser scanner
            %
            % Usage: [scan] = turtlebot.get_scan()
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
        
        function set_wheel_speeds(turtle, W_R, W_L)
            % Set the rotational speed of the left and right wheel
            %
            % Usage: turtlebot.set_wheel_speeds(W_R, W_L)
            %
            % INPUTS:
            %   - W_R = rotational speed of right wheel [rad/s]. A positive
            %           value results in clockwise rotation. A negative
            %           value results in counterclockwise rotation.
            %   - W_L = rotational speed of left wheel [rad/s]. A positive
            %           value results in clockwise rotation. A negative
            %           value results in counterclockwise rotation.
            %
            % OUTPUTS:
            %   - None
            
            % Translate to forward and angular velocity
            v = (turtle.R/2)*(W_R+W_L);
            w = (turtle.R/turtle.B)*(W_R-W_L);
            
            if v >= turtle.v_max
                warning(strcat('Requested linear velocity is larger than maximum, saturating output on: ', num2str(turtle.v_max), ' [m/s]'))
                v = turtle.v_max;
            end
            
            if w >= turtle.w_max
                warning(strcat('Requested angular velocity is larger than maximum, saturating output on: ', num2str(turtle.w_max), ' [rad/s]'))
                w = turtle.w_max;
            end
            
            % Construct Twist message
            turtle.vel_msg.Linear.X = v;
            turtle.vel_msg.Angular.Z = w;
            
            % Publish velocity message
            send(turtle.vel_pub,turtle.vel_msg);
            
        end
        
        function set_linear_angular_velocity(turtle, v, w)
            % Function to apply a linear and angular velocity to the robot
            %
            % Usage: turtlebot.set_linear_angular_velocity(v,w)
            %
            % INPUTS:
            %   - v = linear velocity along the X axis of the robot [m/s].
            %         A positive velocity makes the robot drive forwards. A
            %         negative velocity makes the robot drive backwards.
            %         Maximum speed is 0.2 m/s.
            %   - w = angular velocity around the Z axis of the robot
            %         [rad/s]. A positive velocity makes the robot turn
            %         counterclockwise. A negative velocity makes the robot
            %         turn clockwise.
            % OUTPUTS:
            %   - None
            
            if v >= turtle.v_max
                warning(strcat('Requested linear velocity is larger than maximum, saturating output on: ', num2str(turtle.v_max), ' [m/s]'))
                v = turtle.v_max;
            end
            
            if w >= turtle.w_max
                warning(strcat('Requested angular velocity is larger than maximum, saturating output on: ', num2str(turtle.w_max), ' [rad/s]'))
                w = turtle.w_max;
            end
            
            % Set linear velocity
            turtle.vel_msg.Linear.X = v;
            % Set angular velocity
            turtle.vel_msg.Angular.Z = w;
            
            % Publish velocity message
            send(turtle.vel_pub,turtle.vel_msg);
        end
        
        function set_linear_velocity_radius(turtle, v, r)
            % Moves robot by setting a forward velocity and turning radius.
            %
            % Usage: turtlebot.set_linear_velocity_radius(v, r)
            %
            % INPUTS:
            %   - v = linear velocity along the X axis of the robot [m/s].
            %         A positive velocity makes the robot drive forwards. A
            %         negative velocity makes the robot drive backwards.
            %         Maximum speed is 0.2 m/s.
            %   - r = turning radius around the Z axis of the robot [m]. A
            %         positive radius makes the robot turn counterclockwise.
            %         A negative radius makes the robot turn clockwise.
            % OUTPUTS:
            %   - None
            
            w = v/r;
            
            if w >= turtle.w_max
                warning(strcat('Requested angular velocity is larger than maximum, saturating output on: ', num2str(turtle.w_max), ' [rad/s]'))
                w = turtle.w_max;
            end
            
            % Set linear velocity
            turtle.vel_msg.Linear.X = v;
            % Set angular velocity
            turtle.vel_msg.Angular.Z = w;
            
            % Publish velocity message
            send(turtle.vel_pub,turtle.vel_msg);
        end
        
        function drive_straight(turtle, distance, speed)
            % Let the robot drive a fixed distance at a certain speed
            %
            % Usage: turtlebot.drive_straight(distance, speed)
            %
            % INPUTS:
            %   - distance = distance to travel in meters.
            %   - speed = speed at which to drive in meters per second.
            %             A positive velocity makes the robot drive
            %             forwards. A negative velocity makes the robot
            %             drive backwards. Maximum speed is 0.2 m/s.
            % OUTPUTS:
            %   - None
            
            s = 0;
            turtle.set_linear_angular_velocity(speed, 0);
            
            while abs(s) < distance
                [ds,~] = get_odometry(turtle);
                s = s + ds;
            end
            turtle.stop()
            
        end
        
        function drive_arc(turtle, speed, angle, r)
            % Let the robot drive a fixed distance at a certain speed
            %
            % Usage: turtlebot.drive_arc(speed, angle, r)
            %
            % INPUTS:
            %   - speed = speed at which to drive in meters per second.
            %             A positive velocity makes the robot drive
            %             forwards. A negative velocity makes the robot
            %             drive backwards. Maximum speed is 0.2 m/s.
            %   - angle = angle of arc segment to turn in radians
            %   - r = radius of the arc in meters. A positive radius makes
            %         the robot turn counterclockwise. A negative radius
            %         makes the robot turn clockwise.
            % OUTPUTS:
            %   - None
            
            th = 0;
            turtle.set_linear_velocity_radius(speed, r);
            
            while abs(th) < angle
                [~,dth] = get_odometry(turtle);
                th = th + dth;
            end
            turtle.stop()
            
        end
        
        function turn_angle(turtle, angle, angular_speed)
            % Let the robot turn a fixed angle at a certain angular speed
            %
            % Usage: turn_angle(turtle, angle, angular_speed)
            %
            % INPUTS:
            %   - angle = angle to turn in radians
            %   - angular_speed = speed at which to turn in radians per
            %                     second. A positive velocity makes the
            %                     robot turn counterclockwise. A negative
            %                     velocity makes the robot turn clockwise.
            % OUTPUTS:
            %   - None
            
            th = 0;
            turtle.set_linear_angular_velocity(0, angular_speed);
            
            while abs(th) < angle
                [~,dth] = get_odometry(turtle);
                th = th + dth;
            end
            turtle.stop()
            
        end
        
        function stop(turtle)
            % Set all speeds of the robot to zerp
            %
            % Usage: turtlebot.stop()
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

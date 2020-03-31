%RandomPath Vehicle driver class
%
% Create a "driver" object capable of steering a Vehicle object through random 
% waypoints within a rectangular region and at constant speed.
%
% The driver object is connected to a Vehicle object by the latter's
% add_driver() method.  The driver's demand() method is invoked on every
% call to the Vehicle's step() method.
%
% Methods::
%  init       reset the random number generator
%  demand     return speed and steer angle to next waypoint
%  display    display the state and parameters in human readable form
%  char       convert to string
%      
% Properties::
%  goal          current goal/waypoint coordinate
%  veh           the Vehicle object being controlled
%  dim           dimensions of the work space (2x1) [m]
%  speed         speed of travel [m/s]
%  closeenough   proximity to waypoint at which next is chosen [m]
%
% Example::
%
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%
% Notes::
% - It is possible in some cases for the vehicle to move outside the desired
%   region, for instance if moving to a waypoint near the edge, the limited
%   turning circle may cause the vehicle to temporarily move outside.
% - The vehicle chooses a new waypoint when it is closer than property
%   closeenough to the current waypoint.
% - Uses its own random number stream so as to not influence the performance
%   of other randomized algorithms such as path planning.
%
% Reference::
%
%   Robotics, Vision & Control, Chap 6,
%   Peter Corke,
%   Springer 2011
%
% See also Vehicle.


% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

% TODO
%  should be a subclass of VehicleDriver
%  Vehicle should be an abstract superclass

classdef SquarePath < handle
    properties
        waypoints   % matrix with all waypoints 
        goal        % the next waypoint    
        veh         % the vehicle we are driving
        dim         % the length of the square 
        speed       % speed of travel
        closeenough  % proximity to goal before
        d_prev
        step        % the step size of the waypoints
        step_nr     % the step number
    end

    methods

        function driver = SquarePath(dim, varargin)
        %RandomPath.RandomPath Create a driver object
        %
        % D = RandomPath(DIM, OPTIONS) returns a "driver" object capable of driving 
        % a Vehicle object through random waypoints.  The waypoints are positioned 
        % inside a rectangular region bounded by +/- DIM in the x- and y-directions.
        %
        % Options::
        % 'speed',S      Speed along path (default 1m/s).
        % 'dthresh',D    Distance from goal at which next goal is chosen.
        %
        % See also Vehicle.

        % TODO options to specify region, maybe accept a Map object?

            driver.dim = dim;

            opt.speed = 0.5;
            opt.step = 0.1; %%0.1 is the time step of the vehicle 
            opt.dthresh = opt.step*opt.speed/2;
            opt.waypoints = [zeros(dim/(opt.speed*opt.step),1), [opt.speed*opt.step:opt.speed*opt.step:dim]';
                             - [opt.speed*opt.step:opt.speed*opt.step:dim]',  dim*ones(dim/(opt.speed*opt.step),1);
                             -dim*ones(dim/(opt.speed*opt.step),1),-[-dim+(opt.speed*opt.step):opt.speed*opt.step:0]';
                             [-dim+(opt.speed*opt.step):opt.speed*opt.step:0]',zeros(dim/(opt.speed*opt.step),1) ];
            
            opt = tb_optparse(opt, varargin);

            driver.speed = opt.speed;
            driver.step = opt.step;
            driver.closeenough = opt.dthresh;
            driver.waypoints = opt.waypoints;
            
            drive.d_prev = Inf;

        end

        function init(driver)
        %RandomPath.init Reset random number generator
        %
        % R.init() resets the random number generator used to create the waypoints.
        % This enables the sequence of random waypoints to be repeated.
        %
        % See also RANDSTREAM.
        driver.goal = [];
        driver.step_nr = 0;
        %driver.goal=[0,0]';
        end


        % private method, invoked from demand() to compute a new waypoint
        function setgoal(driver)
            
             if ~isempty(driver.goal)
                 [i] =strmatch(driver.goal', driver.waypoints); %%% find the previous goal 
                 if i==size(driver.waypoints,1) %%% last waypoint
                     driver.goal=driver.waypoints(end,:)';
                     %driver.speed=0; %%% stop the vehicles
                 else 
                     % driver.veh.x(1:2)
                      %if norm(driver.veh.x(1:2)-driver.waypoints(i+1,:)')<norm(driver.veh.x(1:2)-driver.waypoints(i,:)')
                       if colnorm(driver.veh.x(1:2) - driver.goal)< driver.closeenough
                        driver.goal=driver.waypoints(i+1,:)';
                      else
                         driver.goal=driver.waypoints(i,:)'; 
                      end
                 end
             else
                 driver.goal=driver.waypoints(1,:)'; %% take the first waypoint
             end
            
            

        end

        function [speed, steer] = demand(driver)
        %RandomPath.demand Compute speed and heading to waypoint
        %
        % [SPEED,STEER] = R.demand() returns the speed and steer angle to
        % drive the vehicle toward the next waypoint.  When the vehicle is
        % within R.closeenough a new waypoint is chosen.
        %
        % See also Vehicle.
            if isempty(driver.goal)
                driver.setgoal()
            end
     
             speed = driver.speed;  
            % driver.goal
            % driver.veh.x
             %driver.waypoints   
%            

             goal_heading = atan2(driver.goal(2)-driver.veh.x(2), ...
             driver.goal(1)-driver.veh.x(1));
          
          
            %d_heading = angdiff(driver.veh.x(3),goal_heading);
            d_heading = angdiff(goal_heading, driver.veh.x(3));
            % d_heading =   goal_heading - driver.veh.x(3) 
            steer = d_heading;
             
            
            % if nearly at goal point, choose the next one
            d = colnorm(driver.veh.x(1:2) - driver.goal);
            if d < driver.closeenough
                %driver.speed=0      
                driver.setgoal();
            elseif d > driver.d_prev
                driver.setgoal();
            end
            driver.d_prev = d;
            driver.step_nr=driver.step_nr+1;
        end

        function display(driver)
        %RandomPath.display Display driver parameters and state
        %
        % R.display() displays driver parameters and state in compact 
        % human readable form.
        %
        % See also RandomPath.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(driver) );
        end % display()

        function s = char(driver)
        %RandomPath.char Convert to string
        %
        % s = R.char() is a string showing driver parameters and state in in 
        % a compact human readable format. 
            s = 'SquarePath driver object';
            s = char(s, sprintf('  current goal=(%g,%g), dimension %.1f', ...
                driver.goal, driver.dim));
        end

    end % methods
end % classdef

clear all;
close all;

startup_rvc

%% Exercise 1: Dead reckoning

% Making the EKF object
Q=[0.0001^2 0  ; 0 (0.01)^2];  % noise on travelled distance[m] and the heading angle [rad]
veh = Roomba(Q,'x0',[$$],'dt',0.01,'stlim',2*pi);
veh.add_driver(SquarePath(1)); % 1 [m] is the length of the side of the square 
P0=diag([$$].^2); % initial uncertainty of the state
ekf = EKF(veh,Q, P0); 

% Run simulation
ekf.run(800); 

% Plotting
veh.plot_xy('b');
hold on
ekf.plot_xy('r');
ekf.plot_ellipse([], 'g');
legend('Desired path followed by the Roomba','Estimated path based on the odometry');

pause % Stop MATLAB untill a key is pressed
%% Exercise 2: Extended Kalman filter

% Constructing the map
figure()
M=zeros(100,100);
M(100,:)=1; M(1,:)=1; M(:,1)=1;M(:,100)=1;

map = sMap(M,2); % take as features the corners in the map (M) ans as cartesian 
%dimension of the binary map -dim to dim (=2)

map.plot();

% Making the EKF object
V=[0.01^2 0  ; 0 (0.1)^2];  %%% noise on travelled distance and the heading angle 
veh = Roomba(V,'x0',[0,0,pi/2],'dt',0.01,'stlim',2*pi);
veh.add_driver(SquarePath(1)); % 1 [m] is the length of the side of the square 
W = diag([$$].^2); % Measurement noise
sensor = RangeBearingSensor(veh, map, W);
[z,i] = sensor.reading();
ekf = EKF(veh, V, P0, sensor, W, map);

% Run simulation
ekf.run(800); 

% Plotting
veh.plot_xy('b');
hold on
ekf.plot_xy('r');
ekf.plot_ellipse([], 'g');
legend('Desired path followed by the Roomba','Estimated path based on the odometry');
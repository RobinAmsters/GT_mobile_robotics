close all
clear all

%% Parameters
dt = 0.05; % Size of timestepss
n_iterations = 20/dt;
v = 75;
altitude = 1000;

%% Intial state estimate
% Initial state estimate x0 (it is a column vector!!!):
x0=[$$];

% The covariance matric of the uncertainty of the initial state estimate P0:
P0=[$$];

%% Covariance matrices
% Covariance of the state prediction Q:
Q=[$$];

% Covariance matric of the uncertainty on the measurements R:
R=[$$];

%% The state prediction matrix:
F=[$$];


%% Input signals
t = [0:dt:dt*n_iterations];
nsteps=size(t,2);

%% The measurements collected in a matrix with every column a new measurement:
X_real = v*t;
V_real = v*ones(1,nsteps);
Y_real = altitude*ones(1,nsteps);

P=P0;
x=x0;
Xtot=[];
Ptot=[];
Z_measured = ones(1,nsteps);
noise = ones(1,nsteps);

%% Main loop
for i=1:nsteps
    %process model:
    Xtilde = F*x;
    Ptilde = F*P*F'+Q;
    
    %Measurement model:
    [z] = simulate_radar([X_real(i); V_real(i); Y_real(i)], R);
    Z_measured(i) = z;
    [h, H] = radar_model(Xtilde);
    
    % Update step
    K=Ptilde*H'*inv(H*Ptilde*H'+R);
    x= Xtilde + K*(z-h);
    P=(eye(size(K,1),size(K,1))-K*H)*Ptilde;
    
    Xtot = [Xtot,x];
    Ptot=[Ptot diag(P)];
end

%% Plotting

% Position plots
figure(1)
subplot(231)
plot(t,Xtot(1,:))
hold on
plot(t,X_real)
ylabel('Position [m]')
xlabel('Time [s]')
legend('Estimated position','Real position')
grid on

% Velocity plots
subplot(232)
plot(t,Xtot(2,:))
hold on
plot(t,V_real)
ylabel('Velocity [m/s]')
xlabel('Time [s]')
legend('Estimated velocity', 'Real velocity')
grid on

% Altitude plots
subplot(233)
plot(t,Xtot(3,:))
hold on
plot(t,Y_real)
ylabel('Velocity [m/s]')
xlabel('Time [s]')
legend('Estimated altitude', 'Real altitude')
grid on

% Position residual plots
subplot(234)
plot(t,X_real - Xtot(1,:))
ylabel('Position residual [m]')
xlabel('Time [s]')
grid on

% Velocity residual plots
subplot(235)
plot(t,V_real - Xtot(2,:))
ylabel('Velocity residual [m/s]')
xlabel('Time [s]')
grid on

% Velocity residual plots
subplot(236)
plot(t,Y_real - Xtot(3,:))
ylabel('Altitude residual [m]')
xlabel('Time [s]')
grid on

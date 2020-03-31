close all
clear all

%% Parameters
dt = 0.1; % Size of timesteps
t_end = 10; % End time
a = 0.1; % Constant acceleration between time steps
noise = true; % Add noise to measurements
dead_reckoning = false; % Use dead reckoning or Kalman for position estimation


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

%% The measurement state matrix:
H=[$$];% measure position

%% Input signals
u=[a*dt^2/2;a*dt];

t = [0:dt:t_end];
nsteps=size(t,2)-1;

step = a*ones(1,size(t,2))';
ramp = a*t';
quad = 1/2*a*t'.^2;

%% The measurements collected in a matrix with every column a new measurement:
X_real = quad';
V_real = ramp';

if noise
    Z_measured = quad'+rand(1,nsteps+1)*sqrt(R);
else
    Z_measured = quad';
end

P=P0;
x=x0;
Xtot=[x];
Ptot=[diag(P)];

%% Main loop
for i=1:nsteps
    %process model:
    Xtilde = F*x+eye(2)*u;
    Ptilde = F*P*F'+Q;
    
    %Measurement model:
    K =Ptilde*H'*inv(H*Ptilde*H'+R);
    z=Z_measured(i); %select the measurement
    
    x= Xtilde + K*(z-H*Xtilde);
    P=(eye(size(K,1),size(K,1))-K*H)*Ptilde;
    
    if dead_reckoning
        % Dead reckoning override
        x=Xtilde;
        P=Ptilde;
    end
    
    Xtot = [Xtot,x];
    Ptot=[Ptot diag(P)];
end

%% Plotting

% Position plots
figure(1)
subplot(221)
plot(t,Xtot(1,:))
hold on
plot(t,X_real)
plot(t, Z_measured)
ylabel('Position [m]')
xlabel('Time [s]')
legend('Estimated position','Real position','Measured position')
grid on

% Velocity plots
subplot(222)
plot(t,Xtot(2,:))
hold on
plot(t,V_real)
ylabel('Velocity [m/s]')
xlabel('Time [s]')
legend('Estimated velocity', 'Real velocity')
grid on

% Position residual plots
subplot(223)
plot(t,X_real - Xtot(1,:))
ylabel('Position residual [m]')
xlabel('Time [s]')
grid on

% Velocity residual plots
subplot(224)
plot(t,sqrt(Ptot(2,:)))
plot(t,V_real - Xtot(2,:))
ylabel('Velocity residual [m/s]')
xlabel('Time [s]')
grid on

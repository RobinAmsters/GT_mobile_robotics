% Robin Amsters
% C�deric Ketelbuters
% 2015 - 2016

function [Pcov_pred, pose_pred] = motionModel(pose0, Pcov0, ds, dth, rotationMode)
% Calculate the current predicted pose and covariance matrix from previous position and covariance and odometry measurements
%
% INPUTS
% pose0 = previous pose [x;y;theta]
% Pcov0 = previous covariance matrix
% ds = distance returned by getOdometry [m]
% dth = angle returned by getOdometry [rad]
% rotationMode = 0 when robot is turning about its own axis, 1 when it is travelling along an arc
%
% OUTPUTS
% pose_pred = updated pose
% Pcov0 = updated covariance matrix

%% INITIALIZATION
sigma_d = ds*0.02; % uncertainty on odometry distance measurement
sigma_t = dth*0.02; % uncertainty on odometry angle measurement

V = [sigma_d^2 0; 0 sigma_t^2]; % noise matrix on odometry measurements
theta = pose0(3); % get previous angle (incremental)


%% APPLICATION OF MOTION MODEL
if rotationMode == 0 % turning about its own axis
    dpsi = 0; % angle while travelling along an arc = 0
elseif rotationMode == 1 % traveling along an arc
    dpsi = dth;
    dth = 0; % angle while turning about its own axis = 0
end

if ~isnan(ds) && ~isnan(dth)  
    % Predict pose with motion model
    if ds == 0 && dth == 0
        pose_pred = pose0;
    else
        pose_pred = pose0 + [(ds)*cos(theta + dth + dpsi/2);(ds)*sin(theta + dth + dpsi/2); dth + dpsi];
    end
    
    % Make sure angle is smaller than between 0 and 2 pi
%     pose_pred(3) = myWrapTo2Pi(pose_pred(3));
    
    % Jacobians for linearization of covariance matrix
    Fx = [1 0 -ds*sin(theta + dth + dpsi/2); 0 1 ds*cos(theta + dth + dpsi/2); 0 0 1];
    Fv = [cos(theta + dth + dpsi/2) -ds*sin(theta + dth + dpsi/2); sin(theta + dth + dpsi/2) ds*cos(theta + dth + dpsi/2); 0 1];
    
    % Predict covariance matrix
    Pcov_pred = Fx*Pcov0*Fx' + Fv*V*Fv';
    
else
    pose_pred = pose0;
    Pcov_pred = Pcov0;
end
end
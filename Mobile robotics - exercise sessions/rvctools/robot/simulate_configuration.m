function simulate_configuration(config_ID)

show_output = false;

switch config_ID
    case 1
        V = diag([0.01, 2*pi/180].^2);
        bias = 0;
        show_output = true;
    case 2
        V = diag([0.05, 1*pi/180].^2);
        bias = 0;
        show_output = true;
    case 3
        V = diag([0.05, 2*pi/180].^2);
        bias = 0.1;
        show_output = true;
        
end
if show_output
    % Parameters
    x_0 = [0,0,pi/2];
    path_length = 2; % length of square path
    P0 = diag([0.005, 0.005, 0.001].^2); %Initial covariance
    
    veh = Roomba(V,'x0',x_0,'dt',0.01,'stlim',2*pi); %Vehicle object
    veh.add_driver(SquarePath(path_length)); % Let robot drive square path of length 2
    
    ekf = EKF(veh, V, P0); %EKF object
    
    %EKF simulations
    ekf.run(1800);
    
    % Get data
    pose_real = veh.x_hist; % Real pose of the robot
    pose_est = zeros(length(ekf.history), 3); % Estimated pose by EKF
    for i=1:length(ekf.history)
        h = ekf.history(i);
        pose_est(i,:) = h.x_est(1:3)';
    end
    % Add bias;
%     if bias ~= 0
%         i_1 = [x_0(1), x_0(2) + path_length + bias]; % coordinate of first corner point
%         i_2 = [i_1(1) - (path_length + bias), i_1(2)]; % coordinate of second corner point
%         i_3 = [i_2(1), i_2(2)  - (path_length + bias)]; % coordinate of third corner point
%         i_4 = [i_3(1) + path_length + bias, i_3(2)]; % coordinate of fourth corner point
%         pose_est = [x_0(1), x_0(2);
%                     i_1(1), i_1(2);
%                     i_2(1), i_2(2);
%                     i_3(1), i_3(2);
%                     i_4(1), i_4(2);]
%     end
    
    % Plotting
    plot(pose_real(:,1), pose_real(:,2), 'black', 'DisplayName','Real position');
    hold on
    plot(pose_est(:,1), pose_est(:,2),'red', 'DisplayName',strcat('Estimated position (config_{ID} = ', num2str(config_ID),')'));
    grid on
    xlim([-(path_length + 0.5),  0.5])
    ylim([-( 0.5), path_length + 0.5])
    legend('show')
else
    display('WARNING: invalid configuration ID, please select 1, 2 or 3')
end
end
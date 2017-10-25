% Clean up matlab
clearvars -except start goal
close all
clc
startup_rvc
V = diag([0.01, 1*pi/180].^2); %%% noise on travelled distance and the heading angle
% default V = diag([0.01, 1*pi/180].^2);

veh = Roomba(V,'x0',[0,0,pi/2],'dt',0.01,'stlim',2*pi);
veh.add_driver(SquarePath(2));
P0=eye(3)*0.0001;
ekf = EKF(veh, V, P0);
ekf.run(1800);

veh.plot_xy('b');
hold on
ekf.plot_ellipse([], 'g');
legend('Desired path followed by the Robot')
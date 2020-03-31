% Clean up matlab
clearvars -except start goal
close all
clc
startup_rvc;

config_ID = 1;   % Select simulation ID
n_loops = 1;     % Select number of simulations to show

for i=linspace(1,n_loops,n_loops)
    figure()
    simulate_configuration(config_ID);   % show output
end
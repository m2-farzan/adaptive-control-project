clc
clear
clear all

disp('Initializing...')

load_plant_params;
load_controller_params;
load_simulation_params;

generate_disturbance;

[X_0, W_0] = load_initial_state();
S_0 = [X_0, W_0];

disp('Simulating...');

tic;
    [t, S] = ode45(@Sdot, [0, simulation_run_time], S_0);
toc;

X = S(:, 1:7);
W = S(:, 8:18);

disp('Generating plots...')

visualize_results(t, X, W);

disp('Process finished.')
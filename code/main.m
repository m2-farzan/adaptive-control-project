clc
clear
clear all

load_plant_params;
load_controller_params;

[X_0, W_0] = load_initial_state();
S_0 = [X_0, W_0];

tic;
    [t, S] = ode45(@Sdot, [0, 60], S_0);
toc;

X = S(:, 1:7);
W = S(:, 8:18);

visualize_results(t, X, W);

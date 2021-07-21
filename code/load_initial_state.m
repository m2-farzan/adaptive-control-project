function [X_0, W_0] = load_initial_state()
    x = 2;
    y = 2;
    theta = 0;
    v1 = 0.5;
    v2 = 0.5;
    i_a_1 = 2;
    i_a_2 = 2;
    
    X_0 = [x, y, theta, v1, v2, i_a_1, i_a_2];
    
    
    a_hat_1 = 0;
    a_hat_2 = 0;
    a_hat_3 = 0;
    a_hat_4 = 0;
    x_2f = [0, 0];
    x_3f = [0, 0];
    S_11 = 0;
    S_12 = 0;
    S_bar_13 = 0;
    
    W_0 = [a_hat_1, a_hat_2, a_hat_3, a_hat_4, x_2f, x_3f, S_11, S_12, S_bar_13];
end
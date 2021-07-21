function visualize_results(t, X, W)
    % Close all open figures
    close all;

    % Load some global vars (Used to calculate S_13)
    global k_1
    
    % Calculate reference velocities (Used to calculate S_13)
    v_r = zeros(size(t, 1), 1);
    for i =  1:size(t, 1)
        z = z_r(t(i));
        v_r(i) = z(1);
    end

    % Unpack plant states
    x = X(:, 1);
    y = X(:, 2);

    % Unpack controller states
    a_hat_1 = W(:, 1);
    a_hat_2 = W(:, 2);
    a_hat_3 = W(:, 3);
    a_hat_4 = W(:, 4);
    S_11 = W(:, 9);
    S_12 = W(:, 10);
    S_bar_13 = W(:, 11);
    S_13 = S_bar_13 + atan(k_1 * S_12 .* v_r);
    
    % Get controller inputs
    u = zeros(size(t, 1), 2);
    for i =  1:size(t, 1)
        [~, u(i, :)] = Wdot(X(i, :)', W(i, :)', z_r(t(i)));
    end

    % Draw figures
    % Fig 2.A
    fig = figure();
    plot(x, y);
    xlabel('X-axis (m)');
    ylabel('Y-axis (m)');
    saveas(fig, 'fig-2A-xy.eps', 'epsc');
    
    % Fig 2.B
    fig = figure();
    plot(t, S_11);
    xlabel('t (s)');
    ylabel('S_{11} tracking error (m)');
    saveas(fig, 'fig-2B-s11.eps', 'epsc');

    % Fig 2.C
    fig = figure();
    plot(t, S_12);
    xlabel('t (s)');
    ylabel('S_{12} tracking error (m)');
    saveas(fig, 'fig-2C-s12.eps', 'epsc');
    
    % Fig 2.D
    fig = figure();
    plot(t, S_13);
    xlabel('t (s)');
    ylabel('S_{13} tracking error (rad)');
    saveas(fig, 'fig-2D-s13.eps', 'epsc');
    
    % Fig 3.A
    fig = figure();
    plot(t, u);
    xlabel('t (s)');
    ylabel('Control inputs (V)');
    legend('Right wheel', 'Left wheel');
    saveas(fig, 'fig-3A-v.eps', 'epsc');
    
    % Fig 3.B
    fig = figure();
    plot(t, [a_hat_1, a_hat_2]);
    xlabel('t (s)');
    ylabel('Estimated a_1 and a_2');
    legend('a_1', 'a_2');
    saveas(fig, 'fig-3B-a1-a2.eps', 'epsc');
    
    % Fig 3.C
    fig = figure();
    plot(t, a_hat_3);
    xlabel('t (s)');
    ylabel('Estimated a_3');
    saveas(fig, 'fig-3C-a3.eps', 'epsc');
    
    % Fig 3.D
    fig = figure();
    plot(t, a_hat_4);
    xlabel('t (s)');
    ylabel('Estimated a_4');
    saveas(fig, 'fig-3D-a4.eps', 'epsc');

end
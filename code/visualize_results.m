function visualize_results(t, X, W)
    x = X(:, 1);
    y = X(:, 2);
    i_a_1 = X(:, 6);
    i_a_2 = X(:, 7);

    figure();
    plot(x, y);
    xlabel('x');
    ylabel('y');

    figure();
    hold on;
    plot(t, i_a_1, 'DisplayName', 'i_a1');
    plot(t, i_a_2, 'DisplayName', 'i_a2');
    legend();
    xlabel('t');
end
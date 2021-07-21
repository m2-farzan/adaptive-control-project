function z_r = z_r(t)
    % v_r = 0.5;
    % omega_r = 0.5 * sin(t);
    if t < 5
        v_r = 0.25 * (1 - cos(pi*t/5));
        omega_r = 0;
    elseif t < 10
        v_r = 0.5;
        omega_r = 0;
    elseif t < 15
        v_r = 0.25 * (1 + cos(pi*t/5));
        omega_r = 0;
    elseif t < 30
        v_r = 0.25 * (1 - cos(2*pi*t/15));
        omega_r = v_r/5;
    elseif t < 45
        v_r = 0.25 * (1 - cos(2*pi*t/15));
        omega_r = -v_r/5;
    elseif t < 50
        v_r = 0.25 * (1 + cos(pi*t/5));
        omega_r = 0;
    elseif t <= 60
        v_r = 0.5;
        omega_r = 0;
    end
    z_r = [v_r; omega_r];
end
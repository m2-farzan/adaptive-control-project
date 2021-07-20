function [Wdot, u] = Wdot(X, W, z_r)
% Controller states updator.
% This function will update the states of the controller; W contains all controller
% states (W=[]^T) and u_e is the desired control input.
%
%% GLOBAL VARIABLES
% Controller tuning variables
global k_1 k_2 k_3 k_4 k_5 ...
    tau_2 tau_3 ...
    gamma_1 gamma_2 gamma_3 gamma_4 ...
    sigma_1 sigma_2 sigma_3 sigma_4 ...
    delta_1 delta_2

% Plant known variables
global R r

%% UNPACK
% Desired path variables
v_r = z_r(1);
omega_r = z_r(2);

% Plant state variables
x_11 = X(1);
x_12 = X(2);
x_13 = X(3);
x_21 = X(4);
x_22 = X(5);
x_31 = X(6);
x_32 = X(7);

x_2 = [x_21; x_22];
x_3 = [x_31; x_32];

% Controller state variables
a_hat_1 = W(1);
a_hat_2 = W(2);
a_hat_3 = W(3);
a_hat_4 = W(4);
x_2f = W(5:6);
x_3f = W(7:8);
S_11 = W(9);
S_12 = W(10);
S_bar_13 = W(11);

%% STEP 1

% Equation 10
S_dot_11 = r/(2*R) * (x_21 - x_22) * S_12 - r/2 * (x_21 + x_22) + v_r * cos(S_bar_13);
S_dot_12 = -r/(2*R) * (x_21 - x_22) * S_11 + v_r * sin(S_bar_13);
S_dot_13 = omega_r - r/(2*R) * (x_21 - x_22);

% Equation 12
S_13 = S_bar_13 + atan(k_1 * S_12 * v_r);

% Inline between eq. 13 & 15
alpha_1 = (k_1 * v_r^2 * sin(S_bar_13) + k_1 * S_12 * v_r) / (1 + (k_1 * S_12 * v_r)^2); % TODO: Use v_dot_r
% alpha_1 = (k_1 * v_r^2 * sin(S_bar_13) + k_1 * S_12 * 0) / (1 + (k_1 * S_12 * v_r)^2); % TODO: Use non-zero v_dot_r
alpha_2 = integral(@(eta) cos(-atan(k_1 * S_12 * v_r) + eta * S_13), 0, 1);
h_1 = a_hat_1 * v_r * cos(S_bar_13) + k_2 * S_11 + 1/2 * (k_1 * v_r)^4 * S_11^3;
h_2 = (1 + (k_1 * v_r * S_11)/(1 + (k_1 * S_12 * v_r)^2))^(-1) * (a_hat_1 * alpha_2 * v_r * S_12 + a_hat_2 * (omega_r + alpha_1) + k_3 * S_13 + 1/2 * S_13^3);

% Equation 14
x_bar_2 = [h_1 + h_2, h_1 - h_2]';

% Equation 16
x_dot_2f = 1/tau_2 * (-x_2f + x_bar_2);

% Equation 15
a_hat_dot_1 = gamma_1 * (v_r * S_11 * cos(S_bar_13) + alpha_2 * v_r * S_12 * S_13) - sigma_1 * gamma_1 * a_hat_1;
a_hat_dot_2 = gamma_2 * (omega_r + alpha_1) * S_13 - sigma_2 * gamma_2 * a_hat_2;

%% STEP 2
% Equation 17
S_2 = x_2 - x_2f;

% Equation 19
Phi_1 = [-(x_21-x_22)*x_22, 0, -x_21, 0, -x_dot_2f(1), -x_dot_2f(2), 0, 0, 1, 0
         0, (x_21-x_22)*x_21, 0, -x_22, 0, 0, -x_dot_2f(2), -x_dot_2f(1), 0, 1];

% Equation 21
x_bar_3 = -k_4 * S_2 - (a_hat_3 * (Phi_1 * Phi_1') * S_2)/(2 * delta_1^2);

% Equation 22
a_hat_dot_3 = gamma_3 * (S_2' * (Phi_1 * Phi_1') * S_2) / (2 * delta_1^2) - sigma_3 * gamma_3 * a_hat_3;

% Equation 23
x_dot_3f = 1/tau_3 * (x_bar_3 - x_3f);

% Equation 20
Phi_2 = [-x_31, 0, -x_21, 0, -x_dot_3f(1), 0; 0, -x_32, 0, -x_22, 0, -x_dot_3f(2)];

% Equation 24
S_3 = x_3 - x_3f;

% Equation 26
u = -k_5 * S_3 - (a_hat_4 * (Phi_2 * Phi_2') * S_3)/(2 * delta_2^2);

% Equation 27
a_hat_dot_4 = gamma_4 * (S_3' * (Phi_2 * Phi_2') * S_3)/(2 * delta_2^2) - sigma_4 * gamma_4 * a_hat_4;

%% RETURN
Wdot = [
    a_hat_dot_1
    a_hat_dot_2
    a_hat_dot_3
    a_hat_dot_4
    x_dot_2f
    x_dot_3f
    S_dot_11
    S_dot_12
    S_dot_13
];

end

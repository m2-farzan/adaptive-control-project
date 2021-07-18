function Xdot = Xdot(X,u)
% Plant states updator.
% This function will update the states of the plant; X contains all plant
% states (X=[x y theta v1 v2 ia_1 ia_2]^T) and u is the control input (the
% voltage of the motors). 

% Global variables
global R r d I_m I_c I_w m_c m_w tau_d_1 tau_d_2 d_11 d_22 ...
    n_1 n_2 K_t_1 K_t_2 l_a_1 l_a_2 r_a_1 r_a_2 K_e_1 K_e_2 ;

% Assign the input X
% Modeling divide to 3 main part : kinematics and dynamics model of moble
% robot and dynamic model of dc motors
% x_1 = [x y theta] use in kinamatic part
% x_2 = [v1 v2] use in dynamic part 
% x_3 = [i_a_1 i_a_2] use in DC motor modeling part

x_1 = (X(1:3));
x_2 = (X(4:5));
x_3 = (X(6:7));

% calculation  x_1_dot x_2_dot x_3_dot

% calculation  x_1_dot 
J = 0.5*r*[cos(x_1(3)), cos(x_1(3)); sin(x_1(3)), sin(x_1(3)); 1/R , -1/R];

x_1_dot = J*x_2;

% calculation  x_2_dot
I = m_c*(d^2) + 2*m_w*(R^2) + I_c + 2*I_m ; 
m = m_c + 2*m_w ;
m_11 = 0.25*R^(-2)*r^2*(m*R^2 + I) + I_w ;
m_12 = 0.25*R^(-2)*r^2*(m*R^2 - I) ;
M = [m_11, m_12; m_12, m_11];
D = [d_11, 0; 0, d_22];
C = 0.5*1/R*r^2*m_c*d*[0, x_1_dot(3); - x_1_dot(3), 0];
N = diag([n_1 n_2]);
K_T = diag([K_t_1 K_t_2]);
tau_d =[tau_d_1; tau_d_2];

x_2_dot = inv(M)*(-C*x_2 - D*x_2 - tau_d + N*K_T*x_3);

% calculation  x_3_dot
L_a = diag([l_a_1 l_a_2]);
R_a = diag([r_a_1 r_a_2]);
K_E = diag([K_e_1 K_e_2]);

x_3_dot = inv(L_a)*(u - R_a*x_3 - N*K_E*x_2 );

%return the Xdot
Xdot =[x_1_dot; x_2_dot; x_3_dot];end

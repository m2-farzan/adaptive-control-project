clc 
clear all
close all
global R r d I_m I_c I_w m_c m_w tau_d_1 tau_d_2 d_11 d_22 ...
     n_1 n_2 K_t_1 K_t_2 l_a_1 l_a_2 r_a_1 r_a_2 K_e_1 K_e_2 ;

R = 0.75;
r = 0.15;
d = 0.3;
I_m = 0.0025;
I_c = 15.625;
I_w = 0.005;
m_c = 30;
m_w = 1;
tau_d_1 = 0 ;
tau_d_2 = 0;
d_11 = 5 ; 
d_22 = 5 ;
n_1 = 62.55 ;
n_2 = 62.55 ;
K_t_1 = 0.2613;
K_t_2 = 0.2613;
l_a_1 = 0.048;
l_a_2 = 0.048;
r_a_1 = 2.5;
r_a_2 = 2.5;
K_e_1 = 0.2;
K_e_2 = 0.2;

initial = [1.5,1.5,pi/6,0,0,0,0] ;
[t,y]= ode45( @seveneq ,[0,100],initial);
figure
plot(t,y(:,1))
hold on 
plot(t,y(:,2))
hold on
plot(t,y(:,3))
hold on
plot(t,y(:,4))
hold on
plot(t,y(:,5))
hold on
plot(t,y(:,6))
hold on
plot(t,y(:,7))
legend('x','y','\theta','v_1','v_2','i_a1','i_a2')
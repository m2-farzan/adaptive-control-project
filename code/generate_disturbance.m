global tau_d_1 tau_d_2 simulation_run_time
tau_d_1 = normrnd(0,0.5,size(0: 0.1 : simulation_run_time));
tau_d_1(tau_d_1 < -0.5)= -0.5;
tau_d_1(tau_d_1 > 0.5)= 0.5;
tau_d_2 = normrnd(0,0.5,size(0: 0.1 : simulation_run_time));
tau_d_2(tau_d_2 < -0.5)= -0.5;
tau_d_2(tau_d_2 > 0.5)= 0.5;
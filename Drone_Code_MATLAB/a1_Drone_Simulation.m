clear; clc; close all;

choice_TDF = 0; % 0: no TDF;    1: nonrobust;    2: robust;   other: minimax
choice_example = 1; % 0: [0,0,0]->[2,4,10];    1: zig-zag

TDF_Mag_vec = 0;
TDF_T_vec = 0;

[xd_vec,yd_vec,zd_vec] = fct_trajectory(PARAMS.t_vec, choice_example);
states_init = zeros(22,1);

states_init(1:3) = [0;0;5];


[TT, XX] = ode45(@(t,x)fct_ode_nonlinear(t,x,PARAMS.Mquad,PARAMS.mload_nom,PARAMS.L_nom,PARAMS.K_opt,PARAMS.t_vec,xd_vec,yd_vec,zd_vec,TDF_Mag_vec,TDF_T_vec),   PARAMS.t_vec,states_init,PARAMS.ode_optn);

%%% Plotting %%%
fct_plotting(TT,XX,xd_vec,yd_vec,zd_vec)

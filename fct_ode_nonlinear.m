function dydt = fct_ode_nonlinear(t,state,M_hat,m_load,L,K,t_vec,xd_vec,yd_vec,zd_vec,TDF_Mag_vec, TDF_T_vec)
%Controller PID
kpx = K(1);
kpvx = K(2);
kivx = K(3);
kdvx = K(4);

kpy = K(1);
kpvy = K(2);
kivy = K(3);
kdvy = K(4);

kpz = K(5);
kpvz = K(6);
kivz = K(7);
kdvz = K(8);

kpt = K(9);
kptd = K(10);
kitd = K(11);
kdtd = K(12);

kpph = K(9);
kpphd = K(10);
kiphd = K(11);
kdphd = K(12);

kpps = K(13);
kppsd = K(14);
kipsd = K(15);
kdpsd = K(16);

%Constants for quad and load
% b = 0.212132034355964; %length of quadcopter motor arm in metres
% L = 0.760; % Length of load string in metres
g = 9.81; %Gravitational acceleration in m/s^2
Ix =  0.046488641976871; %moment of inertia in kg/m^2
Iy = 0.045121374751049; %moment of inertia in kg/m^2
Iz = 0.069661717120760; %moment of inertia in kg/m^2
l = 0.212132034355964;  %Motor to Centre Length
kt = 1.902033855064101e-05;            %Motor Constant
kb = 3.051539115927946e-07;        %Motor Torque Constant

%states
x_load = state(1);
y_load = state(2);
z_load = state(3);
phi = state(4);
theta = state(5);
psi = state(6);
alp = state(7);
bet = state(8);
vx = state(9);
vy = state(10);
vz = state(11);
phi_dot = state(12);
theta_dot = state(13);
psi_dot = state(14);
alp_dot = state(15);
beta_dot = state(16);
vx_err_int = state(17);
vy_err_int = state(18);
vz_err_int = state(19);
phd_err_int = state(20);
thd_err_int = state(21);
psd_err_int = state(22);

% Position control
% Intermediate Equations
xd = interp1(t_vec, xd_vec, t);
yd = interp1(t_vec, yd_vec, t);
zd = interp1(t_vec, zd_vec, t);

if TDF_Mag_vec == 0
    xd = xd;
    yd = yd;
else
    G = 0;
    G = TDF_Mag_vec(1);
        for ind_Mag = 2:length(TDF_Mag_vec)
            G = G + TDF_Mag_vec(ind_Mag)*heaviside(t-TDF_T_vec(ind_Mag-1));
        end
    xd = G*xd;
    yd = G*yd;
end

x_q = x_load - L*cos(alp)*sin(bet);
y_q = y_load + L*sin(alp);
z_q = z_load - L*cos(alp)*cos(bet);
vx_q = vx - beta_dot*L*cos(alp)*cos(bet) + alp_dot*L*sin(alp)*sin(bet);
vy_q = vy + alp_dot*L*cos(alp);
vz_q = vz + alp_dot*L*sin(alp)*cos(bet) + beta_dot*L*cos(alp)*sin(bet);
x_d = (xd - L*cos(0)*sin(0));
y_d = yd + L*sin(0);
z_d = zd - L*cos(0)*cos(0);



% Velocity control
vz_des = max(min(kpz*(z_d - z_q),1),-1);
if TDF_Mag_vec == 0 
    vx_des = max(min(kpx*(x_d - x_q),1),-1);
    vy_des = max(min(kpy*(y_d - y_q),1),-1);
else
    vx_des = G*max(min(kpx*(x_d - x_q),1),-1);
    vy_des = G*max(min(kpy*(y_d - y_q),1),-1);
end

% old code
% vx_des = kpx*(x_d - x_q);
% vy_des = kpy*(y_d - y_q);
% vz_des = kpz*(z_d - z_q);

Ux_exp1 = m_load + M_hat + m_load*(cos(bet)*cos(bet)*sin(alp)*sin(alp) - cos(bet)*cos(bet) - sin(alp)*sin(alp));
Ux_exp2 = m_load*L*sin(bet)*cos(alp)*(alp_dot*alp_dot + beta_dot*beta_dot*cos(alp)*cos(alp));
Ux = (kpvx*(vx_des - vx_q) + kivx*vx_err_int - kdvx*Ux_exp2/Ux_exp1)/(1/(m_load + M_hat) + kdvx/Ux_exp1);

Uy_exp1 = m_load + M_hat - m_load*cos(alp)*cos(alp);
Uy_exp2 = m_load*L*sin(alp)*(alp_dot*alp_dot + beta_dot*beta_dot*cos(alp)*cos(alp));
Uy = (kpvy*(vy_des - vy_q) + kivy*vy_err_int + kdvy*Uy_exp2/Uy_exp1)/(1/(m_load + M_hat) + kdvy/Uy_exp1);

Uz_exp1 = m_load + M_hat + m_load*(sin(bet)*sin(bet)*sin(alp)*sin(alp) - sin(bet)*sin(bet) - sin(alp)*sin(alp));
Uz_exp2 = m_load*L*cos(bet)*cos(alp)*(alp_dot*alp_dot + beta_dot*beta_dot*cos(alp)*cos(alp));
Uz = (kpvz*(vz_des - vz_q) + kivz*vz_err_int - kdvz*Uz_exp2/Uz_exp1 - (kdvz + 1/(m_load + M_hat))*g)/(1/(m_load + M_hat) + kdvz/Uz_exp1);

T = (m_load + M_hat)*sqrt(Ux^2 + Uy^2 + Uz^2);
% T = sqrt(Ux^2 + Uy^2 + Uz^2);

th_des = atan(Ux/Uz);
ps_des = 0;
ph_des = asin(Uy/T);

%////Inner Loop
th_dot_des = kpt*(th_des - theta);
ps_dot_des = kpps*(ps_des - psi);
ph_dot_des = kpph*(ph_des - phi);

% Fx = -T*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
% Fy = -T*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
% Fz = -T*cos(theta)*cos(phi);
Fx = -T*(cos(ps_des)*sin(th_des)*cos(ph_des) + sin(ps_des)*sin(ph_des));
Fy = -T*(sin(ps_des)*sin(th_des)*cos(ph_des) - cos(ps_des)*sin(ph_des));
Fz = -T*cos(th_des)*cos(ph_des);

taux = (kpphd*(ph_dot_des - phi_dot) + kiphd*phd_err_int)/(1 + kdphd/Ix);
tauy = (kptd*(th_dot_des - theta_dot) + kitd*thd_err_int)/(1 + kdtd/Iy);
tauz = (kppsd*(ps_dot_des - psi_dot) + kipsd*psd_err_int)/(1 + kdpsd/Iz);


%Equations
x_eqn = vx;
y_eqn = vy;
z_eqn = vz;
phi_eqn = phi_dot;
theta_eqn = theta_dot;
psi_eqn = psi_dot;
alp_eqn = alp_dot;
beta_eqn = beta_dot;

% A_mat = [(m_load + M_hat),0,0,0,0,0,L*M_hat*sin(alp)*sin(bet),-L*M_hat*cos(alp)*cos(bet);
%     0,m_load + M_hat,0,0,0,0,L*M_hat*cos(alp),0;
%     0,0,m_load + M_hat,0,0,0,L*M_hat*cos(bet)*sin(alp),L*M_hat*cos(alp)*sin(bet);
%     0,0,0,Ix,0,-Ix*sin(theta),0,0;
%     0, 0,0,0,Iy*cos(phi)^2 + Iz*sin(phi)^2,Iy*cos(phi)*cos(theta)*sin(phi) - Iz*cos(phi)*cos(theta)*sin(phi),0,0;
%     0,0,0, -Ix*sin(theta), Iy*cos(phi)*cos(theta)*sin(phi) - Iz*cos(phi)*cos(theta)*sin(phi), Iz*cos(phi)^2*cos(theta)^2 + Iy*cos(theta)^2*sin(phi)^2 + Ix*sin(theta)^2,0,0;
%     -L*m_load*sin(alp)*sin(bet), -L*m_load*cos(alp), -L*m_load*cos(bet)*sin(alp),0,0,0,0,0;
%     L*m_load*cos(alp)*cos(bet),0,-L*m_load*cos(alp)*sin(bet),0,0,0,0, 0];

A_mat_inv = [-(sin(bet)^2*(sin(alp)^2 - 1))/(M_hat + m_load), -(cos(alp)*sin(alp)*sin(bet))/(M_hat + m_load),      (cos(alp)^2*cos(bet)*sin(bet))/(M_hat + m_load),                                                                                                                                                0,                                                            0,                                                                             0,     -(sin(alp)*sin(bet))/(L*m_load),                   cos(bet)/(L*m_load*cos(alp));
 -(cos(alp)*sin(alp)*sin(bet))/(M_hat + m_load),                    sin(alp)^2/(M_hat + m_load),       -(cos(alp)*cos(bet)*sin(alp))/(M_hat + m_load),                                                                                                                                                0,                                                            0,                                                                             0,                -cos(alp)/(L*m_load),                                              0;
(cos(alp)^2*cos(bet)*sin(bet))/(M_hat + m_load), -(cos(alp)*cos(bet)*sin(alp))/(M_hat + m_load), ((sin(alp)^2 - 1)*(sin(bet)^2 - 1))/(M_hat + m_load),                                                                                                                                                0,                                                            0,                                                                             0,     -(cos(bet)*sin(alp))/(L*m_load),                  -sin(bet)/(L*m_load*cos(alp));
                                              0,                                              0,                                                    0, -(Iy*Iz + Ix*Iy*sin(theta)^2 - Iy*Iz*sin(theta)^2 - Ix*Iy*sin(phi)^2*sin(theta)^2 + Ix*Iz*sin(phi)^2*sin(theta)^2)/(Ix*Iy*Iz*(sin(theta)^2 - 1)), -(cos(phi)*sin(phi)*sin(theta)*(Iy - Iz))/(Iy*Iz*cos(theta)), -(sin(theta)*(Iy - Iy*sin(phi)^2 + Iz*sin(phi)^2))/(Iy*Iz*(sin(theta)^2 - 1)),                                   0,                                              0;
                                              0,                                              0,                                                    0,                                                                                     -(cos(phi)*sin(phi)*sin(theta)*(Iy - Iz))/(Iy*Iz*cos(theta)),                 (Iz + Iy*sin(phi)^2 - Iz*sin(phi)^2)/(Iy*Iz),                                  -(sin(2*phi)*(Iy - Iz))/(2*Iy*Iz*cos(theta)),                                   0,                                              0;
                                              0,                                              0,                                                    0,                                                                    -(sin(theta)*(Iy - Iy*sin(phi)^2 + Iz*sin(phi)^2))/(Iy*Iz*(sin(theta)^2 - 1)),                 -(sin(2*phi)*(Iy - Iz))/(2*Iy*Iz*cos(theta)),              -(Iy - Iy*sin(phi)^2 + Iz*sin(phi)^2)/(Iy*Iz*(sin(theta)^2 - 1)),                                   0,                                              0;
                  (sin(alp)*sin(bet))/(L*M_hat),                             cos(alp)/(L*M_hat),                        (cos(bet)*sin(alp))/(L*M_hat),                                                                                                                                                0,                                                            0,                                                                             0, (M_hat + m_load)/(L^2*M_hat*m_load),                                              0;
                   -cos(bet)/(L*M_hat*cos(alp)),                                              0,                          sin(bet)/(L*M_hat*cos(alp)),                                                                                                                                                0,                                                            0,                                                                             0,                                   0, (M_hat + m_load)/(L^2*M_hat*m_load*cos(alp)^2)];
 

% b_vec =[Fx - m_load*(L*cos(alp)*sin(bet)*alp_dot^2 + 2*L*cos(bet)*sin(alp)*alp_dot*beta_dot + L*cos(alp)*sin(bet)*beta_dot^2) - M_hat*(L*cos(alp)*sin(bet)*alp_dot^2 + 2*L*cos(bet)*sin(alp)*alp_dot*beta_dot + L*cos(alp)*sin(bet)*beta_dot^2) + L*alp_dot^2*m_load*cos(alp)*sin(bet) + L*beta_dot^2*m_load*cos(alp)*sin(bet) + 2*L*alp_dot*beta_dot*m_load*cos(bet)*sin(alp);
    % L*M_hat*sin(alp)*alp_dot^2 + Fy;
    % Fz + g*m_load + g*M_hat - m_load*(L*cos(alp)*cos(bet)*alp_dot^2 - 2*L*sin(alp)*sin(bet)*alp_dot*beta_dot + L*cos(alp)*cos(bet)*beta_dot^2) - M_hat*(L*cos(alp)*cos(bet)*alp_dot^2 - 2*L*sin(alp)*sin(bet)*alp_dot*beta_dot + L*cos(alp)*cos(bet)*beta_dot^2) + L*alp_dot^2*m_load*cos(alp)*cos(bet) + L*beta_dot^2*m_load*cos(alp)*cos(bet) - 2*L*alp_dot*beta_dot*m_load*sin(alp)*sin(bet);
    % taux - (Iy*theta_dot^2*sin(2*phi))/2 + (Iz*theta_dot^2*sin(2*phi))/2 + Ix*psi_dot*theta_dot*cos(theta) - Iy*psi_dot*theta_dot*cos(theta) + Iz*psi_dot*theta_dot*cos(theta) + Iy*psi_dot^2*cos(phi)*cos(theta)^2*sin(phi) - Iz*psi_dot^2*cos(phi)*cos(theta)^2*sin(phi) + 2*Iy*psi_dot*theta_dot*cos(phi)^2*cos(theta) - 2*Iz*psi_dot*theta_dot*cos(phi)^2*cos(theta);
    % tauy + (Ix*psi_dot^2*sin(2*theta))/2 - Ix*phi_dot*psi_dot*cos(theta) + Iy*phi_dot*theta_dot*sin(2*phi) - Iz*phi_dot*theta_dot*sin(2*phi) - Iz*psi_dot^2*cos(phi)^2*cos(theta)*sin(theta) - Iy*psi_dot^2*cos(theta)*sin(phi)^2*sin(theta) - Iy*phi_dot*psi_dot*cos(phi)^2*cos(theta) + Iz*phi_dot*psi_dot*cos(phi)^2*cos(theta) + Iy*phi_dot*psi_dot*cos(theta)*sin(phi)^2 - Iz*phi_dot*psi_dot*cos(theta)*sin(phi)^2;
    % tauz + Ix*phi_dot*theta_dot*cos(theta) - Ix*psi_dot*theta_dot*sin(2*theta) + Iy*theta_dot^2*cos(phi)*sin(phi)*sin(theta) - Iz*theta_dot^2*cos(phi)*sin(phi)*sin(theta) - Iy*phi_dot*theta_dot*cos(phi)^2*cos(theta) + Iz*phi_dot*theta_dot*cos(phi)^2*cos(theta) + Iy*phi_dot*theta_dot*cos(theta)*sin(phi)^2 - Iz*phi_dot*theta_dot*cos(theta)*sin(phi)^2 - 2*Iy*phi_dot*psi_dot*cos(phi)*cos(theta)^2*sin(phi) + 2*Iz*phi_dot*psi_dot*cos(phi)*cos(theta)^2*sin(phi) + 2*Iz*psi_dot*theta_dot*cos(phi)^2*cos(theta)*sin(theta) + 2*Iy*psi_dot*theta_dot*cos(theta)*sin(phi)^2*sin(theta);
    % L*m_load*cos(bet)*sin(alp)*(L*cos(alp)*cos(bet)*alp_dot^2 - 2*L*sin(alp)*sin(bet)*alp_dot*beta_dot + L*cos(alp)*cos(bet)*beta_dot^2) - L*g*m_load*cos(bet)*sin(alp) + L*m_load*sin(alp)*sin(bet)*(L*cos(alp)*sin(bet)*alp_dot^2 + 2*L*cos(bet)*sin(alp)*alp_dot*beta_dot + L*cos(alp)*sin(bet)*beta_dot^2) - L^2*alp_dot^2*m_load*cos(alp)*sin(alp) - L^2*beta_dot^2*m_load*cos(alp)*sin(alp);
    % -L*m_load*cos(alp)*(g*sin(bet) + cos(bet)*(L*cos(alp)*sin(bet)*alp_dot^2 + 2*L*cos(bet)*sin(alp)*alp_dot*beta_dot + L*cos(alp)*sin(bet)*beta_dot^2) - sin(bet)*(L*cos(alp)*cos(bet)*alp_dot^2 - 2*L*sin(alp)*sin(bet)*alp_dot*beta_dot + L*cos(alp)*cos(bet)*beta_dot^2) - 2*L*alp_dot*beta_dot*sin(alp))];

b_vec = [- L*M_hat*cos(alp)*sin(bet)*alp_dot^2 - 2*L*M_hat*cos(bet)*sin(alp)*alp_dot*beta_dot - L*M_hat*cos(alp)*sin(bet)*beta_dot^2 + Fx;
L*M_hat*sin(alp)*alp_dot^2 + Fy;
- L*M_hat*cos(alp)*cos(bet)*alp_dot^2 + 2*L*M_hat*sin(alp)*sin(bet)*alp_dot*beta_dot - L*M_hat*cos(alp)*cos(bet)*beta_dot^2 + Fz + M_hat*g + g*m_load;
taux - (Iy*theta_dot^2*sin(2*phi))/2 + (Iz*theta_dot^2*sin(2*phi))/2 + Ix*psi_dot*theta_dot*cos(theta) - Iy*psi_dot*theta_dot*cos(theta) + Iz*psi_dot*theta_dot*cos(theta) + Iy*psi_dot^2*cos(phi)*cos(theta)^2*sin(phi) - Iz*psi_dot^2*cos(phi)*cos(theta)^2*sin(phi) + 2*Iy*psi_dot*theta_dot*cos(phi)^2*cos(theta) - 2*Iz*psi_dot*theta_dot*cos(phi)^2*cos(theta);
tauy + (Ix*psi_dot^2*sin(2*theta))/2 - Ix*phi_dot*psi_dot*cos(theta) + Iy*phi_dot*theta_dot*sin(2*phi) - Iz*phi_dot*theta_dot*sin(2*phi) - Iz*psi_dot^2*cos(phi)^2*cos(theta)*sin(theta) - Iy*psi_dot^2*cos(theta)*sin(phi)^2*sin(theta) - Iy*phi_dot*psi_dot*cos(phi)^2*cos(theta) + Iz*phi_dot*psi_dot*cos(phi)^2*cos(theta) + Iy*phi_dot*psi_dot*cos(theta)*sin(phi)^2 - Iz*phi_dot*psi_dot*cos(theta)*sin(phi)^2;
tauz + Ix*phi_dot*theta_dot*cos(theta) - Ix*psi_dot*theta_dot*sin(2*theta) + Iy*theta_dot^2*cos(phi)*sin(phi)*sin(theta) - Iz*theta_dot^2*cos(phi)*sin(phi)*sin(theta) - Iy*phi_dot*theta_dot*cos(phi)^2*cos(theta) + Iz*phi_dot*theta_dot*cos(phi)^2*cos(theta) + Iy*phi_dot*theta_dot*cos(theta)*sin(phi)^2 - Iz*phi_dot*theta_dot*cos(theta)*sin(phi)^2 - 2*Iy*phi_dot*psi_dot*cos(phi)*cos(theta)^2*sin(phi) + 2*Iz*phi_dot*psi_dot*cos(phi)*cos(theta)^2*sin(phi) + 2*Iz*psi_dot*theta_dot*cos(phi)^2*cos(theta)*sin(theta) + 2*Iy*psi_dot*theta_dot*cos(theta)*sin(phi)^2*sin(theta);
-L*g*m_load*cos(bet)*sin(alp);
-L*g*m_load*cos(alp)*sin(bet)];

% dot_dot_eqn = A_mat\b_vec

dot_dot_eqn = A_mat_inv*b_vec;

dydt = [x_eqn;y_eqn;z_eqn;phi_eqn;theta_eqn;psi_eqn;alp_eqn;beta_eqn;dot_dot_eqn(:);
    (vx_des - vx_q);(vy_des - vy_q);(vz_des - vz_q);(ph_dot_des - phi_dot);(th_dot_des - theta_dot);(ps_dot_des - psi_dot)];

% t
end
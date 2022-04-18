ts_simulation = 0.01;
Va = 25;



% # Prop parameters
D_prop = 20*(0.0254);  


KV = 145;        
KQ = (1. / KV) * 60. / (2. * pi);
R_motor = 0.042;  
i0 = 1.5;            


ncells = 12;
V_max = 3.7 * ncells ;

C_Q2 = -0.01664;
C_Q1 = 0.004970;
C_Q0 = 0.005230;
C_T2 = -0.1079;
C_T1 = -0.06044;
C_T0 = 0.09357;

V_in = V_max*ts_simulation;

a1 = (rho*C_Q0*D_prop^5)/(2*pi)^2;
b1 = rho*C_Q1*(D_prop^4)*Va/(2*pi) + (KQ^2)/R_motor;
c1 = rho*(D_prop^3)*C_Q2*(Va^2) - KQ*V_in/R_motor + KQ*i0;
Omega_p = (-b1 + sqrt(b1^2 - 4*a1*c1))/(2*a1);

J_op = 2*pi*Va/(Omega_p*D_prop);

% th_a = rho*D_prop^4*C_T0/(4*pi^2);
% th_b = rho*D_prop^3*C_T1*Va/(2*pi);
% th_c = rho*D_prop^2*C_T2*Va^2;
% 
% tr_a = rho*D_prop^5*C_Q0/(4*pi^2);
% tr_b = rho*D_prop^4*C_Q1*Va/(2*pi);
% tr_c = rho*D_prop^3*C_Q2*Va^2;

C_T = C_T2*J_op&2 + C_T1*J_op + C_T0;
C_Q = C_Q2*J_op&2 + C_Q1*J_op + C_Q0;
% 
% thrust_prop = th_a*Omega_p^2 + th_b*Omega_p + th_c
% torque_prop = -tr_a*Omega_p^2 + tr_b*Omega_p + tr_c;

n = Omega_p/(2*pi);
thrust_prop = rho*(n^2)*(D_prop^4)*C_T

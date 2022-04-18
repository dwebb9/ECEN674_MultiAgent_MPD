clear, close all;

pi_long = 3.141592653589793238462643383279;

delta_elevator = -0.1248;
delta_aileron = 0.001836;
delta_rudder = - 0.0003026;
delta_throttle =  0.6768;

ts_simulation =  0.01;

mass = 11;
Jx = 0.8244;
Jy = 1.135;
Jz = 1.759;
Jxz = 0.1204;
S_wing = 0.55;
b = 2.8956;
c = 0.18994;
S_prop = 0.2027;
rho = 1.2682;
e = 0.9;
AR = (b^2) / S_wing;
g = 9.8;

C_L_0 = 0.23;
C_D_0 = 0.043;
C_m_0 = 0.0135;
C_L_alpha = 5.61;
C_D_alpha = 0.03;
C_m_alpha = -2.74;
C_L_q = 7.95;
C_D_q = 0.0;
C_m_q = -38.21;
C_L_delta_e = 0.13;
C_D_delta_e = 0.0135;
C_m_delta_e = -0.99;
M = 50.0;
alpha0 = 0.47;
epsilon = 0.16;
C_D_p = 0.0;

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

alpha = 0;
beta = 0;
Va = 25;
theta = 0;
phi = 0;
q = 0;
r = 0;

a_b = 1 + exp(-M*(alpha-alpha0)) + exp(M*(alpha+alpha0));
b_b =(1 + exp(-M*(alpha-alpha0)))*(1 + exp(M*(alpha+alpha0)));

blend = (a_b)/(b_b);

CL_nonlin = (1-blend)*(C_L_0 + C_L_alpha*alpha) + blend*(2*sign(alpha)*(sin(alpha)^2)*cos(alpha));
CD_nonlin = ((C_L_0 + C_L_alpha*alpha)^2)/(pi*e*AR); % C_D_p = 0

CL = CL_nonlin + C_L_q*c*q/(2*Va) + C_L_delta_e*delta_elevator;
CD = CD_nonlin + C_D_delta_e*delta_elevator; %C_D_q*c*q/(2*Va) = 0


Flift = 0.5*rho*(Va^2)*S_wing*CL;
Fdrag = 0.5*rho*(Va^2)*S_wing*CD

f_longx = -cos(alpha)*Fdrag + sin(alpha)*Flift;

ideal_f_longx = 1.05613943 - 0.9564161283661252

f_x_test = -0.5*rho*(Va^2)*S_wing*CD;

ideal_CD = ideal_f_longx/(-0.5*rho*(Va^2)*S_wing)
CD = ((C_L_0 + C_L_alpha*alpha)^2)/(pi*e*AR) + C_D_delta_e*delta_elevator %C_D_q*c*q/(2*Va) = 0




clear;clc
pn = 4;
Ld = 5.25e-3;
Lq = 12e-3;
R = 0.958;
phi = 0.1827;
J = 0.003;
B = 0.008;

beta = 50;% speed loop bandwidth rad/s
Ba = (beta*J-B)/(1.5*pn*phi);
Kpw = beta*J/(1.5*pn*phi);
Kiw = beta*Kpw;

tao = min(Ld/R,Lq/R);
alpha = 2*pi/tao;
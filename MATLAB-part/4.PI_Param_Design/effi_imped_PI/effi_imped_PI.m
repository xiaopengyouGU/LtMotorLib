clear;clc
pn = 4;
Ld = 8.5e-3;
Lq = 8.5e-3;
R = 2.875;
phi = 0.175;
J = 0.0008;
B = 0;

beta = 90;% speed loop bandwidth rad/s
Ba = (beta*J-B)/(1.5*pn*phi);
Kpw = beta*J/(1.5*pn*phi);
Kiw = beta*Kpw;

alpha = 200;

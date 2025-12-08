clear;clc
wL = 800/(30/pi);
wH = 1300/(30/pi);
tao = 0.516;
J = 4.943e-4;
b = J/tao;
Kt = 0.164;
R = 0.106/2;
L = (0.428e-3)/2;
pn = 2;

TL = 0.3;
IH = (J/tao*wH+TL)/Kt;
IL = (J/tao*wL+TL)/Kt;



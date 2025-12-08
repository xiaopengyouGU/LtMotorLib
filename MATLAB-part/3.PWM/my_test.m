len = 1000;
theta_num = linspace(0,2*pi,len);
% a = 0.1;
% [qU,qV,qW] = spwm(theta_num,theta_num - 2*pi/3, theta_num + 2*pi/3,a);
% plot(theta_num,qU,theta_num,qV,theta_num,qW);
% legend("qU","qV","qW");

% [qU,qV,qW] = spwm_1(theta_num,theta_num - 2*pi/3, theta_num + 2*pi/3,1);
% plot(theta_num,qU,theta_num,qV,theta_num,qW);
% legend("qU","qV","qW");

% [qU,qV,qW] = spwm_2(theta_num,theta_num - 2*pi/3, theta_num + 2*pi/3,1);
% plot(theta_num,qU,theta_num,qV,theta_num,qW);
% legend("qU","qV","qW");

% use u0(000) as zero vector! 
qU = theta_num;
qV = qU;
qW = qV;
Udc = 1;
Ud = 0;
Uq = 0.5;
flag_num = [0 0.5 1];
str_num = ["u0(000)","(u0+u7)/2","u7(111)"];
for k = 1:3
for i = 1:len
    theta_el = theta_num(i);
    [q1,q2,q3] = svpwm(theta_el,Ud,Uq,Udc,flag_num(k));
    qU(i) = q1;
    qV(i) = q2;
    qW(i) = q3;
end
subplot(1,3,k)
plot(theta_num,qU,theta_num,qV,theta_num,qW);
str = sprintf("use %s as zero vector",str_num(k));
title(str);
legend("qU","qV","qW");
end

clear;clc
len = 2000;
The_num = linspace(0,2*pi,len);
EMF_num = The_num;
EMF2_num = The_num;
EMF3_num = The_num;
for i = 1:len
    the = The_num(i);
    EMF_num(i) =BEMF_curve(the);
    EMF2_num(i) = BEMF_curve(mod(the-2*pi/3,2*pi));
    EMF3_num(i) = BEMF_curve(mod(the-4*pi/3,2*pi));
end
subplot(3,1,1)
plot(The_num,EMF_num);
xlabel("theta [rad/s]");
ylabel("back-EMF");
grid on

subplot(3,1,2)
plot(The_num,EMF2_num);
xlabel("theta [rad/s]");
ylabel("back-EMF");
grid on

subplot(3,1,3)
plot(The_num,EMF3_num);
xlabel("theta [rad/s]");
ylabel("back-EMF");
grid on
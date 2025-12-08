len = 2000;
The_num = linspace(0,2*pi,len);
S1_num = The_num;
S2_num = The_num;
S3_num = The_num;
Va_num = The_num;
Vb_num = The_num;
Vc_num = The_num;
dir = 1; %CCW
for i = 1:len
    [s1,s2,s3] = Hall_sensor(The_num(i));
    S1_num(i) = s1;
    S2_num(i) = s2;
    S3_num(i) = s3;
    [Va,Vb,Vc] = Hall_Table(s1,s2,s3,dir);
    Va_num(i) = Va;
    Vb_num(i) = Vb;
    Vc_num(i) = Vc;
end
subplot(3,1,1)
plot(The_num,S1_num,The_num,0.5*Va_num);
xlabel("theta [rad]");
ylabel("signal/Voltage");
legend("s1","Va");
grid on

subplot(3,1,2)
plot(The_num,S2_num,The_num,0.5*Vb_num);
xlabel("theta [rad]");
ylabel("signal/Voltage");
legend("s2","Vb");

grid on
subplot(3,1,3)
plot(The_num,S3_num,The_num,0.5*Vc_num);
xlabel("theta [rad]");
ylabel("signal/Voltage");
legend("s3","Vc");
grid on
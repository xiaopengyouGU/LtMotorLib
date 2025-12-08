clear;clc
syms s kp ki wc w0;

m = s^2 + 2*wc*s + (w0)^2;
den = m;
num = kp*m + 2*ki*wc*s;
den = collect(den,s);
num = collect(num,s);

kp = 1;
w0 = 2*pi*8.5*6;
wc = 5;
ki = 150;
num = [kp, (2*kp*wc + 2*ki*wc), kp*w0^2];
den = [1, (2*wc), w0^2];
G = tf(num,den);

kp_num = [1 10 50];
ki_num = [1 50 100];
wc_num = [pi, 5*pi, 10*pi];
w = logspace(0,5,500);
for k = 1:3
    figure(k);
for i = 1:3
    if k == 1
        kp = kp_num(i);
    elseif k == 2
        ki = ki_num(i);
    else 
        wc = wc_num(i);
    end
    %
    num = [kp, (2*kp*wc + 2*ki*wc), kp*w0^2];
    den = [1, (2*wc), w0^2];
    G = tf(num,den);
    [mag,phase,wout] = bode(G,w);
    [m,n,p] = size(mag);
    mag = reshape(mag,m,n*p);
    phase = reshape(phase,m,n*p);
    %
    subplot(2,1,1)
    mag = 20*log10(mag);
    [m,n,p] = size(mag);
    semilogx(wout,mag);
    xlabel("w [rad/s]");
    ylabel("Mag [dB]");
    hold on
    subplot(2,1,2)
    semilogx(wout,phase);
    xlabel("w [rad/s]");
    ylabel("angle [degree]");
    hold on
end
if k == 1
subplot(2,1,1)
legend("Kp = 1","Kp = 10","Kp = 50");
subplot(2,1,2)
legend("Kp = 1","Kp = 10","Kp = 50");
elseif k == 2
    subplot(2,1,1)
    legend("Ki = 1","Ki = 50","Ki = 100");
    subplot(2,1,2)
    legend("Kp = 1","Ki = 50","Ki = 100");
else
    subplot(2,1,1)
    legend("wc = pi","wc = 5*pi","wc = 10*pi");
    subplot(2,1,2)
    legend("wc = pi","wc = 5*pi","wc = 10*pi");
end
end
%bode(G)
%margin(G);


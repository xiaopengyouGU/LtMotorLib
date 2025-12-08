figure(1)
plot(Nr.time,Nr.signals.values,'b');
grid on
xlabel('Times(s)');
ylabel('Nr(r/min)');

figure(2)
plot(Te.time,Te.signals.values,'k');
grid on
xlabel('Times(s)');
ylabel('Te(N.m)');

figure(3)
plot(Idq.time,Idq.signals.values);
grid on
xlabel('Times(s)');
ylabel('Idq(A)');
legend('Id','Iq');
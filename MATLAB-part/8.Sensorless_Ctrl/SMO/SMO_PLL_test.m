%clear;clc
%out = sim("SMO_PLL_model.slx");
t = out.Nr.time;
Nr = out.Nr.signals.values;
plot(t,Nr(:,1),t,Nr(:,2));
xlabel("time [s]");
ylabel("speed [r/min]");
legend("Nr","Nr_m");
grid on

figure(2)
t = out.Ealpha_beta.time;
Ealpha = out.Ealpha_beta.signals(1).values;
Ebeta = out.Ealpha_beta.signals(2).values;
plot(t,Ealpha,t,Ebeta);
xlabel("time [s]");
ylabel("BEMF [voltage]");
legend("Ealpha","Ebeta");
grid on
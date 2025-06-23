% threshold = 2000;
% Ts = [150 50 50];
% div = [1 1 1];
% raw_data = readmatrix("data_1.csv");
% csv = lt_csv(raw_data,threshold,Ts,div);
% csv.plot(Type="pos_vel",Range=[40 50 -1000 5000],Multi=[1 10 5]);

%threshold = 2000;       % burring datas
%Ts = [100 100];     %channel sample period, unit: ms
%div = [10 1];        %div raw_data to get actual value, eg:channel1 and channel2
% raw_data = readmatrix("stepper_vel.csv");
% csv = lt_csv(raw_data,threshold,Ts,div);
% csv.plot(Type="vel",Range=[76 88 0 820],Multi=[20,1]);
%the code above is equal to the lower codes
% array = csv.get_array();
% arr1 = array(1);
% arr1.y = arr1.y*20;
% arr1.target = arr1.target*20;
% arr2 = array(2);
% plot(arr1.x,arr1.y,"-b",arr1.x,arr1.target,"-r",arr2.x,arr2.y,"-g");
% title("velocity pid");
% xlabel("time [s]");
% ylabel("vel [rpm/20]");
% legend("vel","target","control");
% axis([47.1 61.5 0 820]);
% 

% threshold = 2000;
% Ts = [300 100 100];
% div = [1 10 1];
% raw_data = readmatrix("stepper_pos_vel.csv");
% csv = lt_csv(raw_data,threshold,Ts,div);
% csv.plot(Type="pos_vel",Multi=[1 20 2])
% csv.plot(Type="pos_vel",Range=[40 50 -1000 5000],Multi=[1 10 5]);
% threshold = 2000;
% Ts = [300 100 100];
% div = [1 10 1];
% raw_data = readmatrix("stepper_pos_vel.csv");
% csv = lt_csv(raw_data,threshold,Ts,div);
% csv.plot(Type="pos_vel",Range=[0 20 -200 2300],Multi=[1 20 2])

threshold = 2000;
Ts = [50 50];
div = [1 1];
raw_data = readmatrix("bldc_pos.csv");
csv = lt_csv(raw_data,threshold,Ts,div);
csv.plot(Type="pos",Range=[30 34 -350 1250]);
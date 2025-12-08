function [sys,x0,str,ts] = my_EKF(t,x,u,flag)
    switch flag
        case 0
            [sys,x0,str,ts] = mdlInitiallizeSizes;
        case 2
            sys = mdlUpdate(t,x,u);
        case 3
            sys = mdlOutputs(t,x,u);
        case {1,4,9}
            sys = [];
        otherwise
            error(['Unhandled flag = ',num2str(flag)]);
    end

function [sys,x0,str,ts] = mdlInitiallizeSizes
    sizes = simsizes;%create a struct
    sizes.NumContStates = 0;
    sizes.NumDiscStates = 4;
    sizes.NumOutputs = 4;
    sizes.NumInputs = 4;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    x0 = [0 0 0 0];
    str = [];
    ts = 1e-6;

function sys = mdlUpdate(t,x,u)
persistent P0
    if isempty(P0)
        P0 = diag([0.1, 0.1, 1, 0.01]);
    end

Rs = 2.875;
Ls = 0.0085;
pn = 4;
J = 0.001;
flux = 0.175;
b = 0;
Q = diag([0.1 0.1 1 0.01]);
R = diag([0.2 0.2]);
Ts = 1e-6;
x = x(:);
vs_ab = [u(1) u(2)]';
is_ab = [u(3) u(4)]';
H = [1 0 0 0;0 1 0 0];%C
B = [1/Ls 0 0 0; 0 1/Ls 0 0]';
F = [ -Rs/Ls 0 flux/Ls*sin(x(4)) flux/Ls*x(3)*cos(x(4));
       0     -Rs/Ls   -flux/Ls*cos(x(4)) flux/Ls*x(3)*sin(x(4));
       0      0      0       0;
       0      0      1       0];
%nonlinear function 
f1 = [ -Rs/Ls*x(1)+x(3)*flux/Ls*sin(x(4)); -Rs/Ls*x(2) - x(3)*flux/Ls*cos(x(4));0;x(3)];
%transform function
f2 = diag([1 1 1 1]) + Ts*F;
x = [x(1) x(2) x(3) x(4)]';
X_pred = x + Ts*(f1 + B*vs_ab);%%
Y_pred = H*X_pred;
Y = is_ab;
P_pred = f2*P0*f2' + Q;
K = P_pred*H'*inv(H*P_pred*H'+R);
sys = X_pred + K*(Y-Y_pred);
P0 = P_pred - K*H*P_pred;


function sys = mdlOutputs(t,x,u)
sys = x;

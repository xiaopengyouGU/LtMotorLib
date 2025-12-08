function [sys,x0,str,ts] = sector(t,x,u,flag)
    switch flag
        case 0
            [sys,x0,str,ts] = mdlInitiallizeSizes;
        case 3
            sys = mdlOutputs(t,x,u);
        case {2,4,9}
            sys = [];
        otherwise
            error(['Unhandled flag = ',num2str(flag)]);
    end

function [sys,x0,str,ts] = mdlInitiallizeSizes
    sizes = simsizes;%create a struct
    sizes.NumContStates = 0;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 1;
    sizes.NumInputs = 2;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    x0 = [];
    str = [];
    ts = [-1 0];

function sys = mdlOutputs(t,x,u)
if(u(1) == 0)
    N = 1;
else
    % inv_Clark transform
    a1 = u(1);
    b1 = u(1)*(-0.5) + (sqrt(3)/2)*u(2);
    c1 = u(1)*(-0.5) - (sqrt(3)/2)*u(2);
    if a1 > 0
        a = 0;
    else
        a = 1;
    end
    if b1 > 0
        b = 0;
    else
        b = 1;
    end
    if c1 > 0
        c = 0;
    else
        c = 1;
    end
    N = 4*a + 2*b + c; % sector calculation
end
sys = N;

function [qU,qV,qW] = spwm(theta_U,theta_V,theta_W,a)
    if a > 1 || a < 0
        fprintf("input error: a must in 0~1!!! \n");
        return;
    end
    qU = 1/2 + a/2*sin(theta_U);
    qV = 1/2 + a/2*sin(theta_V);
    qW = 1/2 + a/2*sin(theta_W);
end
    
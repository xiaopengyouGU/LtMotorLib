function [qU,qV,qW] = spwm_1(theta_U,theta_V,theta_W,a)
    Up = -min(min(a*sin(theta_U),a*sin(theta_V)),a*sin(theta_W)) - 1;
    qU = 1/2 + 1/2*(a*sin(theta_U) + Up);
    qV = 1/2 + 1/2*(a*sin(theta_V) + Up);
    qW = 1/2 + 1/2*(a*sin(theta_W) + Up);
end
    
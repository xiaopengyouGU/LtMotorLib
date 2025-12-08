function [qU,qV,qW] = svpwm(theta_el,Ud,Uq,Udc,flag)
    vec_sector = [4 6 
                  6,2
                  2,3
                  3,1
                  1,5
                  5,4];
    % inverse Park transform
    ca = cos(theta_el);
    sa = sin(theta_el);
    U_alpha = ca*Ud - sa*Uq;
    U_beta = sa*Ud + ca*Uq;
    % % inverse Clark transform
    % Ua = U_alpha;
    % Ub = -0.5*U_alpha + sqrt(3)/2*U_beta;
    % Uc = -0.5*U_alpha - sqrt(3)/2*U_beta;
    % sector calculation
    sector =  fix(theta_el/(pi/3)) + 1;
    if sector > 6 
        sector = 6;
    end
    U1 = vec_sector(sector,1);
    U2 = vec_sector(sector,2);
    theta = theta_el - (sector - 1)*pi/3;
    Uout = abs(U_alpha + j*U_beta);
    q1 = sqrt(3)*Uout/Udc*sin(pi/3 - theta);
    q2 = sqrt(3)*Uout/Udc*sin(theta);
    if flag == 0% use u0 as zero vector          
        qU = bitget(U1,3)*q1 + bitget(U2,3)*q2;
        qV = bitget(U1,2)*q1 + bitget(U2,2)*q2;
        qW = bitget(U1,1)*q1 + bitget(U2,1)*q2;
    elseif flag == 1%use u7 as zero vector
        q3 = 1 - q1 - q2;
        qU = bitget(U1,3)*q1 + bitget(U2,3)*q2 + q3;
        qV = bitget(U1,2)*q1 + bitget(U2,2)*q2 + q3;
        qW = bitget(U1,1)*q1 + bitget(U2,1)*q2 + q3;
    else% use (u0 + u7)/2 as zero vector
        q3 = (1 - q1 - q2)/2;
        qU = bitget(U1,3)*q1 + bitget(U2,3)*q2 + q3;
        qV = bitget(U1,2)*q1 + bitget(U2,2)*q2 + q3;
        qW = bitget(U1,1)*q1 + bitget(U2,1)*q2 + q3;
    end
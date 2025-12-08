function [b0,b1,b2,a1,a2] = PR_controller(w0,Ki,wc,Ts)
    b0 = 4*Ki*wc*Ts/(4+4*wc*Ts+w0^2*Ts^2);
    b1 = 0;
    b2 = -4*Ki*wc*Ts/(4+4*wc*Ts+w0^2*Ts^2);
    a1 = (2*w0^2*Ts^2 - 8)/(4+4*wc*Ts+w0^2*Ts^2);
    a2 = (4-4*wc*Ts + w0^2*Ts^2)/(4+4*wc*Ts+w0^2*Ts^2);
end
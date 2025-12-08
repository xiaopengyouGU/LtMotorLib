function y = BEMF_curve(u)
if u <= pi/6
    y = u;
elseif u < 5/6*pi
    y = pi/6;
elseif u < 7/6*pi
    y = pi - u;
elseif u < 11/6*pi
    y = -pi/6;
else 
    y = u - 2*pi;
end

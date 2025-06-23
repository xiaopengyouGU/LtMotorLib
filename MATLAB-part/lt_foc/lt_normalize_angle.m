function angle_el = lt_normalize_angle(angle)
    n = angle/(2*pi);
    n = abs(fix(n));
    if angle <= 0
        angle = angle + (n+1)*2*pi;
    else
        angle = angle - n*2*pi;
    end
    angle_el = angle;
end
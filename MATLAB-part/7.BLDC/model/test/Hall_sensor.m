function [s1,s2,s3] = Hall_sensor(u)
    u = mod((u - pi/6),2*pi);
    sector = fix(u/(pi/3)) + 1;
    if sector >= 6
        sector = 6;
    end
    Hall_Tab = [1 0 1;1 0 0;1 1 0;0 1 0; 0 1 1;0 0 1];
    s1 = Hall_Tab(sector,1);
    s2 = Hall_Tab(sector,2);
    s3 = Hall_Tab(sector,3);
end
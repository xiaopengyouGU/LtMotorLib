function relay_table(I,u)
if I(1) == 0 && I(2) == 0 && I(3) == 0
    u = [0,0,0,0,0,0];
elseif I(1) == 0
    if u(2) > 0
            u = [0,0,1,0,0,1];%normal operate
    else
            u = [0,0,0,1,1,0];%change sector
    end
elseif I(2) == 0
     if u(1) > 0
            u = [1,0,0,0,0,1];
     else
            u = [0,1,0,0,1,0];
     end
else
    % sector 1
     if u(1) > 0
            u = [1,0,0,1,0,0];%normal operate
     else
            u = [0,1,1,0,0,0];%normal operate
     end
end
%
g1 = u(1);
g2 = u(2);
g3 = u(3);
g4 = u(4);
g5 = u(5);
g6 = u(6);
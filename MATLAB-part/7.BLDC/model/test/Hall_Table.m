function [A,B,C] = Hall_Table(s1,s2,s3,dir)
    states = s1*4 + s2*2 + s3*1;
    if states == 0 || states == 7
        A = 0;
        B = 0;
        C = 0; % error Hall signals, stop motor
        return;
    end
    %map table
    map_tab = [6, 4, 5, 2, 1, 3];
    six_commut_tab = [1,-1,0; 1,0,-1;0,1,-1;-1,1,0;-1,0,1;0,-1,1];
    if dir > 0 %CCW, forward rotation
        index = map_tab(states);
    else %CW, reversal rotation
        index = mod(map_tab(states) - 1 + 3,6) + 1;
    end
    A = six_commut_tab(index,1);
    B = six_commut_tab(index,2);
    C = six_commut_tab(index,3);
end
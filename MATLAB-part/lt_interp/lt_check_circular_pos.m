function res =  lt_check_circular_pos(ps,pe)
        tmp = norm(ps);
        ref = tmp/100;
        tmp = tmp - norm(pe);
        if abs(tmp) > ref
            res = 0;
        else 
            res = 1;
        end
end
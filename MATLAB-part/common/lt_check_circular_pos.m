function res =  lt_check_circular_pos(start,target)
        tmp = norm(start);
        ref = tmp/100;
        tmp = tmp - norm(target);
        if abs(tmp) > ref
            res = 0;
        else 
            res = 1;
        end
end
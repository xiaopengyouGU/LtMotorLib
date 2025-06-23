function res = lt_check_end(p,pe,bias)
    dist = norm(pe-p);
    res = 0;
    if bias == 1
        if dist == 0
            res = 1;
        end
    else
        if dist <= 2  
            res = 1;
        end
    end
end
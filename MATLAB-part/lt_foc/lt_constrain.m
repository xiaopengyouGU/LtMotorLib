function res = lt_constrain(val,min,max)
    if min > max 
        res = 0;
        return;
    end
    if val < min
        val = min;
    elseif val > max
        val = max;
    end
    res = val;
       
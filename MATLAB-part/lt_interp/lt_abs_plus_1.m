function res =  lt_abs_plus_1(x,dir)
            if x < 0
                x = x - 1;
            elseif x > 0
                x = x + 1;
            else
                if dir > 0
                x = x + 1;
                else
                x = x - 1;
                end
            end
            res = x;
end



    

    function res =  lt_get_quard(pos,dir)
	    if pos(1) > 0
		    if pos(2) > 0	
                res =  1;		
            elseif pos(2) < 0	
                res = 4;	
            else
			    if dir > 0
                    res = 1;
                else
     			   res = 4;
                end
            end
        elseif pos(1) < 0
            if pos(2) > 0
                res = 2;
            elseif pos(2) < 0
                res = 3;
            else
                if dir > 0
                    res = 3;
                else
                    res = 2;
                end
            end
        else
            if pos(2) > 0
                if dir > 0
                    res = 2;
                else
                    res = 1;
                end
            else
                 if dir > 0
                    res = 4;
                else
                    res = 3;
                 end
            end
        end
end

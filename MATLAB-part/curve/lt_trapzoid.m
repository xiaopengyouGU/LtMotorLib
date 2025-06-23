function [t,freq,T_nums] = lt_trapzoid(step,acc,dec,speed)

	T_nums = zeros(1,step);
    t = zeros(1,step);
    freq = T_nums;
    %get T0
    T0 = 0.69*sqrt(1000 * 2 / acc) * 1000;% unit:us
    t(1) = T0/0.69;
    freq(1) = 1e6/T0;
    %
	acc_step = (dec * step)/(acc + dec);
	acc_max_step = (speed * speed)/(2 * acc) /1000;			
	acc_step = round(acc_step);
    acc_max_step = round(acc_max_step);

	if acc_step <= acc_max_step				
		dec_step = step - acc_step;
	else									
		dec_step = round((acc_max_step * acc)/dec);
        acc_step = acc_max_step;
    end

	if dec_step == 0								
		dec_step = 1;
    end
	dec_start = step - dec_step;
	
	T_nums(1) = T0;		
	for i = 2:acc_step							
		T_nums(i) = T_nums(i-1) - 2*T_nums(i-1)/(4*(i-1) + 1);
        t(i) = T_nums(i) + t(i-1);
        freq(i) = 1e6/T_nums(i);
    end
	for i = (acc_step + 1):dec_start								
		T_nums(i) = T_nums(i-1);
        t(i) = T_nums(i) + t(i-1);
        freq(i) = 1e6/T_nums(i);
    end
	for i = (dec_start + 1):step					
		j = step + 1 - i;
		T_nums(i) = T_nums(i-1) + 2*T_nums(i-1)/(4*j - 1);
        t(i) = T_nums(i) + t(i-1);
        freq(i) = 1e6/T_nums(i);
    end
    t = t./1e6;
end

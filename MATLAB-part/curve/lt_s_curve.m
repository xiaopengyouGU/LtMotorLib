function[t,freq,T_nums] = lt_s_curve(step,acc_step,dec_step,freq_min,freq_max,flexible)
	delt_f = freq_max - freq_min;
	freq = zeros(1,step);
    T_nums = zeros(1,step);
    t_sum = 0;
	dec_start = step - dec_step;	

	for i = 1:acc_step			
		num = flexible * (i - acc_step/2)/(acc_step/2);
		den = 1.0 + exp(-num);
		f = freq_min + delt_f/den;
		T_nums(i) = 1000000/f;
        freq(i) = f;
        t_sum = t_sum + T_nums(i);
    end
	for i = (acc_step+1): dec_start
		T_nums(i) = T_nums(i-1);
        freq(i) = freq(i-1);
        t_sum = t_sum + T_nums(i);
    end
	for i = (dec_start+1):step
		j = i-(dec_start+1);
		num = flexible * ( j - dec_step/2)/(dec_step/2);
        den = 1.0 + exp(-num);
		f = freq_max - delt_f/den;
		T_nums(i) = 1000000/f;
        freq(i) = f;
        t_sum = t_sum + T_nums(i);
    end
    t = linspace(0,step/1000,step);
end
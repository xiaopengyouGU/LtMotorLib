function[t,freq,T_nums] = lt_5_section(step,acc_t,vo,vt)
    if vo < vt
        is_inc = 1;
    else
        tmp = vo;
        vo = vt;
        vt = tmp;
        is_inc = 0;
    end

	acc_t = acc_t/2;		
	J =  (vt - vo)/(acc_t * acc_t);
	inc_step = (vo * acc_t + 1.0/6 * J * acc_t * acc_t * acc_t)/1000;
    inc_step = round(inc_step);
	dec_step = ((vt + vo) * acc_t)/1000 - inc_step;	
    dec_step = round(dec_step);
	acc_step = inc_step + dec_step;
    
    if step < 2*acc_step
        step = 2*acc_step;
    end

    dec_start = step - acc_step;
	T_nums = zeros(1,step);
    t = T_nums;
    freq = t;

	T0 = power(1000*6.0/J,1/3);									
	sum_t = T0;												
	delt_v = 0.5 * J * power(sum_t,2);
	vel = vo + delt_v;
    t(1) = sum_t;
	T_nums(1) = 1000/vel;		
    freq(1) = 1000/T_nums(1);
	
	for i = 2:acc_step										
		Ti = T_nums(i-1);											
		sum_t = sum_t + Ti;
        t(i) = t(i-1) + Ti;
		if i <= inc_step
			delt_v = 0.5 * J * power(sum_t,2);
			vel = delt_v + vo;
			if i == inc_step	
				sum_t = abs(sum_t - acc_t);
            end
        else
			delt_v = 0.5 * J * power(abs(acc_t - sum_t),2);
			vel = vt - delt_v;
        end
		T_nums(i) = 1000/vel;
    end
    % if descend 
    if is_inc ~= 1
        for i = 1:acc_step/2
            tmp = freq(i);
            freq(i) = freq(acc_step + 1 - i);
            freq(acc_step + 1 -i) = tmp;
            tmp1 = T_nums(i);
            T_nums(i) = T_nums(acc_step + 1- i);
            T_nums(acc_step + 1 - i) = tmp1;
        end
        t(1) = T_nums(i);
        for i = 2:acc_step
            t(i)  = t(i-1) + T_nums(i);
        end
    end
    % const speed part
    if acc_step < dec_start
    for i = (acc_step+1):(dec_start+1)
        T_nums(i) = T_nums(i-1);
        t(i) = t(i-1) + T_nums(i);
    end
    end
    % decelerate part
    for i = (dec_start+1):step
        T_nums(i) = T_nums(step-i+1);
        t(i) = t(i-1) + T_nums(i);
    end

	for i = 1:step
		T_nums(i) = 1000*T_nums(i);	
        freq(i) = 1e6/T_nums(i);
    end
    t = t./1000;
end

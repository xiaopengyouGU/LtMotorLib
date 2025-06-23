classdef lt_interp < handle
    properties
        pos;
        target;
        devia;
        type;
        dir;
        num_pulse;
        num_pulse_a;
    end
    properties(Access = private)
        map_table = [0x2,	 0x1,	0x3,	0x0;
					 0x3,	 0x2,	0x0,	0x1;
					 0x0,	 0x3,	0x1,	0x2;
					 0x1,	 0x0,	0x2,	0x3]
        desired_path;
        path;
        x_dir;
        y_dir;
        bias;
    end
    methods
        function interp = lt_interp(start,target,dir,type)
            assert(size(start,1) == 2,"should input 2X1 column");
            assert(size(target,1) == 2,"should input 2X1 column");
            interp.devia = 0;
            interp.target = abs(target - start);
            interp.type = type;
            bias = [0 0]';
            if type == 1        %circular interp
                res = lt_check_circular_pos(start(1),start(2),target(1),target(2));
                assert(res == 1,"two pos aren't in the same circular");
                interp.num_pulse = 65535;
                interp.target = target;
                interp.dir = dir;
            else
                if norm(start) ~= 0
                    bias = start;
                end
                interp.num_pulse = interp.target(1) + interp.target(2);
                interp.desired_path = [start target];
                %set interp line dir
                set_line_dir(interp,target-start);
            end
            interp.pos = start - bias;
            interp.path = start - bias;
            interp.num_pulse_a = interp.num_pulse;
            interp.bias = bias;
        end

        function res = lt_process(interp)
            devia1 = interp.devia;
            pos1 = interp.pos;
            target1 = interp.target;
            if interp.type == 1    %circular interp
                if norm(interp.pos - interp.target) == 0%%finish interp
                    res = 0;
                    return;
                end
                x = lt_get_quard(interp.pos,interp.dir);
                if interp.dir > 0% ccw
                    if devia1 >= 0
                        y = 1;
                    else
                        y = 2;
                    end
                else
                    if devia1 >= 0
                        y = 3;
                    else
                        y = 4;
                    end
                end

                val = interp.map_table(x,y);
               move_axis = bitget(val,1);

                if move_axis == 0%x move
                    if devia1 >= 0
                        devia1 = devia1 - 2*abs(pos1(1)) + 1;
                        pos1(1) = lt_abs_sub_1(pos1(1));
                    else
                        devia1 = devia1 + 2*abs(pos1(1)) + 1;
                        pos1(1) = lt_abs_plus_1(pos1(1),0);
                    end
                else	%y move
                    if devia1 >= 0
                        devia1 = devia1 - 2*abs(pos1(2)) + 1;
                        pos1(2) = lt_abs_sub_1(pos1(2));
                    else
                        devia1 = devia1 + 2*abs(pos1(2)) + 1;
                        pos1(2) = lt_abs_plus_1(pos1(2),0);
                    end
                end
                interp.devia = devia1;
            else% line interp
                if interp.num_pulse == 0
                    res = 0;
                    return;
                end

                num_pulse1 = interp.num_pulse - 1;
                % pos 1
                interp.num_pulse = num_pulse1;
                if devia1 >= 0
                    interp.devia = devia1 - target1(2);
                    pos1(1) = lt_abs_plus_1(pos1(1),interp.x_dir);
                else
                    interp.devia = devia1 + target1(1);
                    pos1(2) = lt_abs_plus_1(pos1(2),interp.y_dir);
                end
            end
            %pos 2
            interp.path = [interp.path pos1];
            interp.pos = pos1;
            res = 1;
        end
        
        function res = process(interp)
            interp.num_pulse = interp.num_pulse_a;
            interp.devia = 0;
            clf;
            res = lt_process(interp);
            while res == 1
                res = lt_process(interp);
            end
            interp.path = interp.path + interp.bias;
            plot(interp.desired_path(1,:),interp.desired_path(2,:));
            axis equal;
            hold on;
            plot(interp.path(1,:),interp.path(2,:));
            legend("desired path","interp path");
            
        end
        % interp test
        function res = test(interp,start,target)
            if interp.type ~= 1 % line interp
            interp.pos = [0 0]';
            interp.target = abs(target-start);
            tmp = target - start;
            interp.num_pulse = abs(tmp(1)) + abs(tmp(2));
            interp.num_pulse_a = interp.num_pulse;
            interp.path = [0 0]';
            interp.desired_path = [start,target];
            set_line_dir(interp,tmp);
            interp.bias = start;
            else
                interp.pos = start;
                interp.target = target;
            end
            process(interp);
            res = 1;
        end

        function res = set_line_dir(interp,pos)
                if pos(1) > 0
                   interp.x_dir = 1;
                else
                    interp.x_dir = 0;
                end
                if pos(2) > 0
                    interp.y_dir = 1;
                else 
                    interp.y_dir = 0;
                end
        end
        end
end

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

   function res = lt_abs_sub_1(x)
            if x < 0
                x = x + 1;
            else
                x = x - 1;
            end
            res = x;
   end

   function path = lt_two_points_r_circular(P1, P2, r, dir)
  %should input 2X1 column vector
    mid = (P1 + P2)/2;
    dirVec = P2 - P1;
    dist = norm(dirVec);
    
    if dist > 2*r
        error('radius is too small ');
    end
    
    %calculate normal vector
    perpVec = [-dirVec(2), dirVec(1)]' / dist;
    
    % select center based on dir--> 0: clockwise, 1: counterclockwise;
    h = sqrt(r^2 - (dist/2)^2);
    if dir == 0
        center = mid + h * perpVec;
    else
        center = mid - h * perpVec;
    end
    
    % calculate relative angle;
    theta1 = atan2(P1(2)-center(2), P1(1)-center(1));
    theta2 = atan2(P2(2)-center(2), P2(1)-center(1));
    
    % guarantee angel sequence
    if theta2 < theta1
        theta2 = theta2 + 2*pi;
    end
    
    % generate points
    theta = linspace(theta1, theta2, 100);
    x = center(1) + r * cos(theta);
    y = center(2) + r * sin(theta);
    
    path = [x;y];
    % % 绘制
    % plot(x, y, 'b-', 'LineWidth', 1);
    % hold on;
    % plot(P1(1), P1(2), 'ro', P2(1), P2(2), 'ro');
    % axis equal;
    % grid on;
end
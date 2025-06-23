classdef lt_interp < handle
    properties
        ps; % start pos
        pe; % end pos
        r;  % radius
        dir; % circular interp dir--> 0: clockwise, 1:counter-clockwise;
    end
    properties(Access = private)
        map_circle = [0x0,	 0x3,	0x1,	0x2;
					 0x1,	 0x0,	0x2,	0x3;
					 0x2,	 0x1,	0x3,	0x0;
					 0x3,	 0x2,	0x0,	0x1];
        map_line = [0x3
                    0x2
                    0x0
                    0x1];
        desired_path;
        path;
        pos;    %%current pos, standard
        trans;  %line interp: relative abs pos, circular interp: circle center
        num_pulse;
        devia; %deviation 
        quadrant; % current quadrant
        target;
        exact; %1: can reach target, 0: have little bias
    end
    methods
        function interp = lt_interp()
            interp.exact = 0;
        end

        function [res,pos] = calc(interp)
            if interp.r == 0
                [res,pos] = calc_line(interp);
            else
                [res,pos] = calc_circular(interp);
            end
        end
        
        function[path,desired_path] = process(interp,ps,pe,r,dir)
            pre_process(interp,ps,pe,r,dir); 
            path = interp.pos;
            while 1
                [res,pos] = calc(interp);
                 path = [path pos];
                if res == 1
                    break;
                end
            end
            interp.path = path + interp.trans;
            path = interp.path;
            desired_path = interp.desired_path;
        end
        %plot path and desired_path
        function plot(interp)
            clf;
            plot(interp.desired_path(1,:),interp.desired_path(2,:));
            axis equal;
            hold on;
            plot(interp.path(1,:),interp.path(2,:));
            legend("desired path","interp path");
        end
    end

    methods(Access=private)
        function pre_process(interp,ps,pe,r,dir)
            assert(size(ps,1) == 2,"should input 2X1 column");
            assert(size(pe,1) == 2,"should input 2X1 column");
            interp.devia = 0;
            %record basic info
            interp.ps = ps;
            interp.pe = pe;
            interp.dir = dir;
            interp.r = r;
            if r == 0        %line interp
                interp.trans = ps;
                interp.pos = ps - interp.trans;
                interp.target = pe - interp.trans;
                interp.quadrant = lt_get_quard(interp.target,0);
                interp.desired_path = [ps pe];
                P = abs(interp.target);
                interp.num_pulse = P(1) + P(2);
            else             %circular interp
                center = lt_get_center(ps,pe,r,dir);
                if norm(center - fix(center)) <= 1e-8 %avoid little calculation bias
                    interp.exact = 1;             %can reach target accuracy
                else
                    interp.exact = 0;
                end
                interp.trans = fix(center);
                interp.pos = ps - interp.trans;
                interp.target = pe - interp.trans;
                interp.desired_path = lt_two_points_r_circular(ps,pe,r,dir);
            end
        end
        function [move_dir] = get_line_dir(interp)
            val = interp.map_line(interp.quadrant);
            if interp.devia >= 0
                val = bitget(val,1);
            else
                val = bitget(val,2);
            end
            move_dir = val;
        end

        function [move_dir,move_axis] = get_circular_dir(interp)
            x = lt_get_quard(interp.pos,interp.dir);
            dev = interp.devia;
            if interp.dir > 0 % ccw
                if dev >= 0
                    y = 1;
                else
                    y = 2;
                end
            else               %cw
                if dev >= 0
                    y = 3;
                else
                    y = 4;
                end
            end
            val = interp.map_circle(x,y);
            move_axis = bitget(val,1);
            move_dir = bitget(val,2);
        end


        function [res,P] = calc_line(interp)
            P = interp.pos;
            Pe = abs(interp.target);
            res = 0;
            if interp.num_pulse == 0
                res = 1;
                return;
            end
            interp.num_pulse = interp.num_pulse - 1;
            %get move_dir
            move_dir = get_line_dir(interp);
            if interp.devia >= 0
                interp.devia = interp.devia - Pe(2);
                P(1) = lt_abs_plus_1(P(1),move_dir);
            else
                interp.devia = interp.devia + Pe(1);
                P(2) = lt_abs_plus_1(P(2),move_dir);
            end
            interp.pos = P;
        end

        function [res,P] = calc_circular(interp)
            P = interp.pos;
            res = lt_check_end(P,interp.target,interp.exact);
            if res == 1% reach end pos successfully
                return;
            end
            %get move dir and move axis
            dev = interp.devia;
           [move_dir,move_axis] = get_circular_dir(interp);
            if move_axis == 0%x move
                if dev >= 0
                    dev = dev - 2*abs(P(1)) + 1;
                    P(1) = lt_abs_sub_1(P(1));
                else
                    dev = dev + 2*abs(P(1)) + 1;
                    P(1) = lt_abs_plus_1(P(1),move_dir);
                end
            else	%y move
                if dev >= 0
                    dev = dev - 2*abs(P(2)) + 1;
                    P(2) = lt_abs_sub_1(P(2));
                else
                    dev = dev + 2*abs(P(2)) + 1;
                    P(2) = lt_abs_plus_1(P(2),move_dir);
                end
            end

            interp.devia = dev;
            interp.pos = P;
        end
    end
end


classdef lt_foc < handle
    properties
        fa;         %A phase
	    fb;         %B phase
	    fc;	        %C phase 
	    max_val;			
	    type;		%foc type
    end
    properties(Access = private)
        theta;      %electric angle
        phases;     %three phases value
    end
    methods
        function foc = lt_foc(max_val,type)
            foc.max_val = abs(max_val);
            foc.type = type;
            foc.fa = 0;
            foc.fb = 0;
            foc.fc = 0;
        end
        function[fa,fb,fc] = calc(foc,fd,fq,angle_el)
            %check angle_el
            angle_el = lt_normalize_angle(angle_el);
            center = foc.max_val/2;
            max = foc.max_val/2;
            fq = lt_constrain(fq,-max,max);
            fd = lt_constrain(fd,-max,max);
            ca = cos(angle_el);
            sa = sin(angle_el);

            % Park inverse transform
            f_alpha = ca*fd - sa*fq;
            f_beta  = sa*fd + ca*fq;
            % Clark inverse transform
            foc.fa = f_alpha;
            foc.fb = -0.5*f_alpha + sqrt(3)/2*f_beta;
            foc.fc = -0.5*f_alpha - sqrt(3)/2*f_beta;

            if foc.type == 1 			% we use center modulation,SVPWM
                u_min = min(foc.fa,min(foc.fb,foc.fc));
                u_max = max(foc.fa,max(foc.fb,foc.fc));
                center = center - (u_max + u_min)/2;
            end
            foc.fa = lt_constrain(foc.fa + center,0,foc.max_val);
            foc.fb = lt_constrain(foc.fb + center,0,foc.max_val);
            foc.fc = lt_constrain(foc.fc + center,0,foc.max_val);
            fa = foc.fa;
            fb = foc.fb;
            fc = foc.fc;
        end
        function [theta,phases] = process(foc,fd,fq,dir)
            if dir > 0
            theta = linspace(0,6*pi,200);
            else
                theta = linspace(-6*pi,0,200);
            end
            phaseA = zeros(size(theta));
            phaseB = phaseA;
            phaseC = phaseB;
            for i = 1:size(theta,2)
                [a,b,c] = calc(foc,fd,fq,theta(i));
                phaseA(i) = a;
                phaseB(i) = b;
                phaseC(i) = c;
            end
            phases = [phaseA;phaseB;phaseC]; %3XN matrix
            foc.phases = phases;
            foc.theta = theta;
        end
        function plot(foc)
            assert(size(foc.phases,1) == 3);
            assert(size(foc.phases,2) > 1);
            assert(size(foc.theta,2) > 1);
            theta = foc.theta;
            clf;
            plot(theta,foc.phases(1,:),theta,foc.phases(2,:),theta,foc.phases(3,:));
            legend("fa","fb","fc");
            axis([min(theta) max(theta)  0 foc.max_val]);
        end
    end
end
       
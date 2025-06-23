classdef lt_csv < handle
    properties
        threshlod;
        Ts;
    end
    properties(Access=private)
        data;
        array;
        array_raw;
    end
    methods
        function csv = lt_csv(raw_data,threshold,Ts,multi)
            csv.data = raw_data;
            csv.threshlod = threshold;
            csv.Ts = Ts;
            csv.burring(threshold);
            csv.array_raw = csv.separate(); %save raw array;
            csv.x_trans();                  % x axis unit transform
            csv.div(multi);
        end
    end

    methods(Access=private)
        function burring(csv,threshold)
            raw_data = csv.data;
            row = size(raw_data,1);
            bias_prev = raw_data(2,2) - raw_data(2,1);
            for i = 2 : row - 1
                bias1 = raw_data(i,2) - raw_data(i-1,2);
                bias2 = raw_data(i,2) - raw_data(i+1,2);
                if abs(bias1) > threshold && abs(bias2) > threshold
                    raw_data(i,2) = (raw_data(i-1,2) + raw_data(i+1,2))/2;
                    bias_prev = raw_data(i,2) - raw_data(i-1,2);
                elseif abs(bias1) > threshold
                    raw_data(i,2) = raw_data(i-1,2) + bias_prev;
                elseif abs(bias1) < threshold && abs(bias2) < threshold
                    bias_prev = raw_data(i,2) - raw_data(i-1,2);
                end
            end
            csv.data = raw_data;
            %the end of each channel data is NAN! so burring algorithm can work well
        end

        function array = separate(csv)
            data = csv.data;
            k = find(data(:,1) == 1);
            num = size(k,1);
            len = size(data,1);
            k = [k;len];

            array = struct();
            for i = 1:num
                array(i).x = data(k(i):k(i+1)-1,1);
                array(i).y = data(k(i):k(i+1)-1,2);
                array(i).target = data(k(i):k(i+1)-1,3);
            end
            csv.array = array;
        end

        function x_trans(csv)
            len = size(csv.array,2);
            Ts = csv.Ts/1000;  % ms --> s
            for i = 1:len
                arr = csv.array(i);
                if size(Ts,2) == 1
                    arr.x = arr.x .*Ts(1);
                else
                    arr.x = arr.x .*Ts(i);
                end
                csv.array(i) = arr;
            end
        end
        function div(csv,multi)
        for i = 1:size(multi,2)
                csv.array(i).y = csv.array(i).y/multi(i);
             end
        end
    end

    methods
        function res = plot(csv,i,options)
            arguments
                csv;
                i double = 1;
                options.Type string = "default"
                options.Range (1,4) = [-1 -1 -1 -1];
                options.Multi (1,:) = [1 1 1 1];
            end
            clf;
            arr = csv.array(i);
            if size(arr,1) == 0
                res = 0;
                return;
            end
            mul = options.Multi;
            str_y = "output";
            plot(arr.x,mul(1)*arr.y,"-b",arr.x,mul(1)*arr.target,"-r");
            if options.Type == "default"
                title(sprintf("Channel%d data",i));
                legend("acctual","target");
            elseif options.Type == "vel" || options.Type == "pos" || options.Type == "current"
                arr1 = csv.array(1);
                arr2 = csv.array(2);
                clf;
                plot(arr1.x,mul(1)*arr1.y,"-b",arr1.x,mul(1)*arr1.target,"-r",arr2.x,mul(2)*arr2.y,"-g");
                if options.Type == "vel"
                    if mul(1) ~= 1
                        str_y = sprintf("vel [rpm/%.0f]",mul(1));
                    else
                        str_y = "vel [rpm]";
                    end
                    title("velocity pid ");
                    legend("vel","target","control");
                elseif options.Type == "pos"
                    if mul(1) ~= 1
                        str_y = sprintf("pos [degree/%.0f]",mul(1));
                    else
                        str_y = "pos [degree]";
                    end
                    title("position pid");
                    legend("pos","target","control");
                else
                    if mul(1) ~= 1
                        str_y = sprintf("current [A/%.0f]",mul(1));
                    else
                        str_y = "current [A]";
                    end
                    title("current pid");
                    legend("current","target","control");
                end
            elseif options.Type == "pos_vel"
                arr1 = csv.array(1);
                arr2 = csv.array(2);
                arr3 = csv.array(3);
                clf;
                plot(arr1.x,mul(1)*arr1.y,"-b",arr1.x,mul(1)*arr1.target,"-r",arr2.x,mul(2)*arr2.y,"-k",arr3.x,mul(3)*arr3.y,"-g");
                legend("pos","target","vel","control");
                title("position velocity loop");
                if mul(1) ~= 1 && mul(2) ~= 1
                    str_y = sprintf("pos [degree/%.0f]; vel [rpm/%.0f]; output",mul(1),mul(2));
                elseif mul(1) ~= 1
                    str_y = sprintf("pos [degree/%.0f]; vel [rpm]; output",mul(1));
                elseif mul(2) ~= 1
                    str_y = sprintf("pos [degree]; vel [rpm/%.0f]; output",mul(2));
                else
                    str_y = "pos [degree]; vel [rpm]; output";
                end
            end
            if options.Range(1) ~= -1
                axis(options.Range);
            end
            xlabel("time [s]");
            ylabel(str_y);
            res = 1;
        end

        function plot_raw(csv,i)
            clf;
            arr = csv.array_raw(i);
            plot(arr.x,arr.y,"-b",arr.x,arr.target,"-r");
            title(sprintf("Channel%d data",i))
            xlabel("sample point");
            ylabel("output");
            legend("acctual","target");
        end

        function array = get_array(csv)
            array = csv.array;
        end
    end
end

% ps = [-5 -5]';
% interp.process(ps,[10 5]',0,0);
% ps = [5 5]';
% pe = [-10 -5]';
% interp.process(ps,pe,0,0);
% ps = [10 -5]';
% pe = [-5 5]';
% interp.process(ps,pe,0,0);
% ps = [-10 5]';
% pe = [5 -5]';
% interp.process(ps,pe,0,0);

%circular interp
ps = [0 10]';
pe = [10 0]';
r = 10;
dir = 1;% counter-clockwise
[path,desired_path] = interp.process(ps,pe,r,1);
interp.plot();
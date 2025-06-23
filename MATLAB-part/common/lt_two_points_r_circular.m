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
  function path = lt_two_points_r_circular(P1, P2, r, dir)
  %should input 2X1 column vector
    %get circle center
    center = lt_get_center(P1,P2,r,dir);
    % calculate relative angle;
    theta1 = atan2(P1(2)-center(2), P1(1)-center(1));
    theta2 = atan2(P2(2)-center(2), P2(1)-center(1));
    
    %guarantee angel sequence
    if dir == 0
    if theta1 < theta2
         theta1 = theta1 + 2*pi;
    end
    else
        if theta2 < theta1
         theta2 = theta2 + 2*pi;
        end
    end
    
    % generate points
    theta = linspace(theta1, theta2, 100);
    x = center(1) + r * cos(theta);
    y = center(2) + r * sin(theta);
    
    path = [x;y];
    % % plot
    % plot(x, y, 'b-', 'LineWidth', 1);
    % hold on;
    % plot(P1(1), P1(2), 'ro', P2(1), P2(2), 'ro');
    % axis equal;
    % grid on;
end
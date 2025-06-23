function center = lt_get_center(P1,P2,r,dir)
%get center of a circle
mid = (P1 + P2)/2;
    dirVec = P2 - P1;
    dist = norm(dirVec);
    
    if dist > 2*r
        error('radius is too small ');
    end
    
    %calculate normal vector
    perpVec = [-dirVec(2), dirVec(1)]' / dist;
    
    % select center based on dir--> 0: clockwise, 1: counter-clockwise;
    h = sqrt(r^2 - (dist/2)^2);
    if dir == 1
        center = mid + h * perpVec;
    else
        center = mid - h * perpVec;
    end
end
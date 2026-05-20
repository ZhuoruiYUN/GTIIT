
function update_spring(spring_line, fixed_points, x1, y1, y2)
% UPDATE_SPRING 
%   input：
%       spring_line: figrue handle
%       fixed_points: handle array


    % spring parameters 
    numLoops = 4;
    width = 0.1;
    resolution = 10;
    
    % calculate length
    l = y1 - y2;
    
    % calculate position
    [springX, springY] = calculate_spring_coords(x1, y1, l, numLoops, width, resolution);
    
    % update springs' line
    set(spring_line, 'XData', springX, 'YData', springY);
    
    % update the fixed points
    set(fixed_points(1), 'XData', x1, 'YData', y1);
    set(fixed_points(2), 'XData', x1, 'YData', y2);
end
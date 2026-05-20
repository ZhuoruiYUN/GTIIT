function [spring_line, fixed_points] = init_spring(x1, y1, x2, y2)
    
    numLoops = 4;       % number loops
    width = 0.1;        % spring width
    resolution = 10;    % line segments
    
    
    [springX, springY] = calculate_spring_coords(x1, y1, x2, y2, numLoops, width, resolution);
    
    
    spring_line = plot(springX, springY, 'b-', 'LineWidth', 2);
    
    
    fixed_points(1) = plot(x1, y1, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    fixed_points(2) = plot(x2, y2, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    
end
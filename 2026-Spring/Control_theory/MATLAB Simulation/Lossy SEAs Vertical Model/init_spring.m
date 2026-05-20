function [spring_line, fixed_points] = init_spring(x1, y1, y2)
% INIT_SPRING 
%   inputs：
%       x1, x location
%       y1: y1 location
%       y2: y2 location
%   outputs：
%       spring_line: handle
%       fixed_points: handle array

    % springs' parameters
    numLoops = 5;       % number loops
    width = 0.03;        % spring width
    resolution = 10;    % line segments
    
    % calculate the length
    l = y1 - y2;
    
    % spring locations
    [springX, springY] = calculate_spring_coords(x1, y1, l, numLoops, width, resolution);
    
    % spring line
    spring_line = plot(springX, springY, 'b-', 'LineWidth', 2);
    
    % fixed point
    hold on;
    fixed_points(1) = plot(x1, y1, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    fixed_points(2) = plot(x1, y2, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    hold off;
end
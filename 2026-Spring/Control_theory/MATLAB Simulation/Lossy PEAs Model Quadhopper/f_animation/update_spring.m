function update_spring(spring_line, fixed_points, x1, y1, x2, y2)
    
    numLoops = 4;
    width = 0.1;
    resolution = 10;
    
    
    [springX, springY] = calculate_spring_coords(x1, y1, x2, y2, numLoops, width, resolution);
    
    
    set(spring_line, 'XData', springX, 'YData', springY);
    
    
    set(fixed_points(1), 'XData', x1, 'YData', y1);
    set(fixed_points(2), 'XData', x2, 'YData', y2);
end